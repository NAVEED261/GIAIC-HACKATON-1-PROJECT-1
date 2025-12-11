"""
Tests for RAG service integration.

Tests the complete RAG pipeline: embedding, search, generation, storage.
"""

import pytest
from unittest.mock import AsyncMock, MagicMock, patch
from sqlalchemy.ext.asyncio import AsyncSession

from app.services.rag_service import RAGService
from app.services.embedding_service import EmbeddingService
from app.services.qdrant_service import QdrantService
from app.services.chat_service import ChatService


@pytest.fixture
def mock_embedding_service():
    """Create mock embedding service."""
    service = AsyncMock(spec=EmbeddingService)
    service.embed_query = AsyncMock(return_value=[0.1] * 1536)
    return service


@pytest.fixture
def mock_qdrant_service():
    """Create mock Qdrant service."""
    service = AsyncMock(spec=QdrantService)
    service.search = AsyncMock(return_value=[
        {
            "id": "test-id-1",
            "score": 0.92,
            "payload": {
                "text": "ROS 2 is the next generation Robot Operating System...",
                "chapter": "ROS 2 Fundamentals",
                "module": 1,
                "week": 3,
                "file_path": "docs/week-03/ros2-fundamentals.md"
            }
        },
        {
            "id": "test-id-2",
            "score": 0.85,
            "payload": {
                "text": "ROS 2 differs from ROS 1 in several key ways...",
                "chapter": "Architecture Overview",
                "module": 1,
                "week": 2,
                "file_path": "docs/week-02/architecture.md"
            }
        }
    ])
    return service


@pytest.fixture
def mock_chat_service():
    """Create mock chat service."""
    service = AsyncMock(spec=ChatService)
    service.generate_answer = AsyncMock(return_value={
        "answer": "ROS 2 is an open-source framework for robot development...",
        "tokens_used": 150,
        "model": "gpt-4o-mini"
    })
    return service


@pytest.fixture
def rag_service(mock_embedding_service, mock_qdrant_service, mock_chat_service):
    """Create RAGService with mocked dependencies."""
    return RAGService(
        embedding_service=mock_embedding_service,
        qdrant_service=mock_qdrant_service,
        chat_service=mock_chat_service
    )


@pytest.mark.asyncio
async def test_generate_answer_complete_flow(
    rag_service: RAGService,
    test_db: AsyncSession,
    sample_session_id: str,
    sample_user_query: str
):
    """Test complete RAG pipeline from query to answer."""
    result = await rag_service.generate_answer(
        query=sample_user_query,
        session_id=sample_session_id,
        db=test_db
    )

    # Verify result structure
    assert "answer" in result
    assert "sources" in result
    assert "session_id" in result
    assert "confidence" in result

    # Verify answer
    assert result["answer"] == "ROS 2 is an open-source framework for robot development..."
    assert result["session_id"] == sample_session_id

    # Verify sources
    assert len(result["sources"]) == 2
    assert result["sources"][0]["chapter"] == "ROS 2 Fundamentals"
    assert result["sources"][0]["score"] == 0.92

    # Verify confidence (should be max score)
    assert result["confidence"] == 0.92


@pytest.mark.asyncio
async def test_generate_answer_stores_conversation(
    rag_service: RAGService,
    test_db: AsyncSession,
    sample_session_id: str,
    sample_user_query: str
):
    """Test that conversation is stored in database."""
    from sqlalchemy import select
    from app.db.models import ChatSession, ChatMessage

    await rag_service.generate_answer(
        query=sample_user_query,
        session_id=sample_session_id,
        db=test_db
    )

    # Verify session was created
    result = await test_db.execute(
        select(ChatSession).where(ChatSession.session_id == sample_session_id)
    )
    session = result.scalar_one_or_none()
    assert session is not None
    assert session.session_id == sample_session_id

    # Verify messages were stored
    result = await test_db.execute(
        select(ChatMessage).where(ChatMessage.session_id == sample_session_id)
    )
    messages = result.scalars().all()

    assert len(messages) == 2

    # Check user message
    user_msg = messages[0]
    assert user_msg.role == "user"
    assert user_msg.content == sample_user_query
    assert user_msg.sources == []

    # Check assistant message
    assistant_msg = messages[1]
    assert assistant_msg.role == "assistant"
    assert assistant_msg.content == "ROS 2 is an open-source framework for robot development..."
    assert len(assistant_msg.sources) == 2
    assert assistant_msg.confidence == 0.92
    assert assistant_msg.tokens_used == 150


@pytest.mark.asyncio
async def test_generate_answer_with_chat_history(
    rag_service: RAGService,
    test_db: AsyncSession,
    sample_session_id: str,
    mock_chat_service: AsyncMock
):
    """Test that chat history is passed to chat service."""
    from app.db.models import ChatSession, ChatMessage

    # Create existing session with history
    session = ChatSession(session_id=sample_session_id)
    test_db.add(session)

    # Add previous messages
    msg1 = ChatMessage(
        session_id=sample_session_id,
        role="user",
        content="What is ROS?"
    )
    msg2 = ChatMessage(
        session_id=sample_session_id,
        role="assistant",
        content="ROS is Robot Operating System..."
    )
    test_db.add_all([msg1, msg2])
    await test_db.commit()

    # Generate new answer
    await rag_service.generate_answer(
        query="Tell me more about it",
        session_id=sample_session_id,
        db=test_db
    )

    # Verify chat service was called with history
    call_args = mock_chat_service.generate_answer.call_args
    assert call_args is not None

    chat_history = call_args.kwargs["chat_history"]
    assert len(chat_history) >= 2
    assert chat_history[0]["role"] == "user"
    assert chat_history[0]["content"] == "What is ROS?"


@pytest.mark.asyncio
async def test_generate_answer_no_results(
    rag_service: RAGService,
    mock_qdrant_service: AsyncMock,
    test_db: AsyncSession,
    sample_session_id: str
):
    """Test RAG pipeline when no search results found."""
    # Mock empty search results
    mock_qdrant_service.search = AsyncMock(return_value=[])

    result = await rag_service.generate_answer(
        query="Some unrelated query",
        session_id=sample_session_id,
        db=test_db
    )

    # Should still generate answer (with no context)
    assert result["answer"] is not None
    assert result["sources"] == []
    assert result["confidence"] == 0.0


@pytest.mark.asyncio
async def test_get_chat_history(
    rag_service: RAGService,
    test_db: AsyncSession,
    sample_session_id: str
):
    """Test retrieving chat history."""
    from app.db.models import ChatSession, ChatMessage

    # Create session with messages
    session = ChatSession(session_id=sample_session_id)
    test_db.add(session)
    await test_db.flush()

    messages_data = [
        {"role": "user", "content": "Question 1"},
        {"role": "assistant", "content": "Answer 1", "sources": [{"chapter": "Test", "score": 0.9}]},
        {"role": "user", "content": "Question 2"},
        {"role": "assistant", "content": "Answer 2", "sources": [{"chapter": "Test2", "score": 0.8}]}
    ]

    for msg_data in messages_data:
        msg = ChatMessage(session_id=sample_session_id, **msg_data)
        test_db.add(msg)

    await test_db.commit()

    # Retrieve history
    history = await rag_service.get_chat_history(
        session_id=sample_session_id,
        db=test_db
    )

    assert len(history) == 4
    assert history[0]["role"] == "user"
    assert history[1]["role"] == "assistant"
    assert history[1]["sources"][0]["chapter"] == "Test"


@pytest.mark.asyncio
async def test_format_sources(rag_service: RAGService):
    """Test source formatting from search results."""
    search_results = [
        {
            "id": "1",
            "score": 0.95,
            "payload": {
                "chapter": "Test Chapter",
                "module": 2,
                "week": 5,
                "text": "Some content"
            }
        }
    ]

    sources = rag_service._format_sources(search_results)

    assert len(sources) == 1
    assert sources[0]["chapter"] == "Test Chapter"
    assert sources[0]["module"] == 2
    assert sources[0]["week"] == 5
    assert sources[0]["score"] == 0.95


@pytest.mark.asyncio
async def test_generate_answer_service_calls(
    rag_service: RAGService,
    mock_embedding_service: AsyncMock,
    mock_qdrant_service: AsyncMock,
    mock_chat_service: AsyncMock,
    test_db: AsyncSession,
    sample_session_id: str,
    sample_user_query: str
):
    """Test that all services are called correctly in RAG pipeline."""
    await rag_service.generate_answer(
        query=sample_user_query,
        session_id=sample_session_id,
        db=test_db
    )

    # Verify embedding service called
    mock_embedding_service.embed_query.assert_called_once_with(sample_user_query)

    # Verify Qdrant search called
    mock_qdrant_service.search.assert_called_once()
    search_call = mock_qdrant_service.search.call_args
    assert len(search_call.kwargs["query_vector"]) == 1536

    # Verify chat service called
    mock_chat_service.generate_answer.assert_called_once()
    chat_call = mock_chat_service.generate_answer.call_args
    assert chat_call.kwargs["query"] == sample_user_query
    assert len(chat_call.kwargs["context_chunks"]) == 2
