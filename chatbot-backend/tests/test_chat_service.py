"""
Tests for OpenAI chat service.

Tests chat completion generation with context and history.
"""

import pytest
from unittest.mock import AsyncMock, MagicMock, patch

from app.services.chat_service import ChatService


@pytest.fixture
def mock_openai_client():
    """Create mock OpenAI client."""
    with patch("app.services.chat_service.AsyncOpenAI") as mock_client:
        yield mock_client.return_value


@pytest.fixture
def chat_service(mock_openai_client):
    """Create ChatService with mocked client."""
    service = ChatService()
    service.client = mock_openai_client
    return service


@pytest.fixture
def sample_context_chunks():
    """Provide sample context chunks from search."""
    return [
        {
            "id": "1",
            "score": 0.92,
            "payload": {
                "text": "ROS 2 is the next generation Robot Operating System with improved architecture.",
                "chapter": "ROS 2 Fundamentals",
                "module": 1,
                "week": 3,
                "file_path": "docs/week-03/ros2-fundamentals.md"
            }
        },
        {
            "id": "2",
            "score": 0.85,
            "payload": {
                "text": "Key improvements include DDS middleware and better real-time support.",
                "chapter": "Architecture Overview",
                "module": 1,
                "week": 2,
                "file_path": "docs/week-02/architecture.md"
            }
        }
    ]


@pytest.mark.asyncio
async def test_generate_answer_basic(
    chat_service: ChatService,
    mock_openai_client,
    sample_user_query: str,
    sample_context_chunks
):
    """Test basic answer generation with context."""
    # Mock OpenAI response
    mock_response = MagicMock()
    mock_response.choices = [
        MagicMock(message=MagicMock(content="ROS 2 is an improved version of ROS..."))
    ]
    mock_response.usage.total_tokens = 200

    mock_openai_client.chat.completions.create = AsyncMock(return_value=mock_response)

    # Generate answer
    result = await chat_service.generate_answer(
        query=sample_user_query,
        context_chunks=sample_context_chunks,
        chat_history=None
    )

    # Verify result
    assert result["answer"] == "ROS 2 is an improved version of ROS..."
    assert result["tokens_used"] == 200
    assert result["model"] == "gpt-4o-mini"

    # Verify API was called
    mock_openai_client.chat.completions.create.assert_called_once()


@pytest.mark.asyncio
async def test_generate_answer_with_history(
    chat_service: ChatService,
    mock_openai_client,
    sample_context_chunks
):
    """Test answer generation with chat history."""
    # Mock response
    mock_response = MagicMock()
    mock_response.choices = [
        MagicMock(message=MagicMock(content="Sure, let me explain more..."))
    ]
    mock_response.usage.total_tokens = 250

    mock_openai_client.chat.completions.create = AsyncMock(return_value=mock_response)

    # Chat history
    chat_history = [
        {"role": "user", "content": "What is ROS 2?"},
        {"role": "assistant", "content": "ROS 2 is..."}
    ]

    # Generate answer
    result = await chat_service.generate_answer(
        query="Tell me more",
        context_chunks=sample_context_chunks,
        chat_history=chat_history
    )

    assert result["answer"] == "Sure, let me explain more..."

    # Verify messages include history
    call_args = mock_openai_client.chat.completions.create.call_args
    messages = call_args.kwargs["messages"]

    # Should have: system prompt + history (2 msgs) + current query
    assert len(messages) >= 4
    assert messages[0]["role"] == "system"
    assert messages[1]["role"] == "user"
    assert messages[1]["content"] == "What is ROS 2?"


@pytest.mark.asyncio
async def test_generate_answer_empty_context(
    chat_service: ChatService,
    mock_openai_client,
    sample_user_query: str
):
    """Test answer generation with no context chunks."""
    mock_response = MagicMock()
    mock_response.choices = [
        MagicMock(message=MagicMock(content="I don't have specific information..."))
    ]
    mock_response.usage.total_tokens = 100

    mock_openai_client.chat.completions.create = AsyncMock(return_value=mock_response)

    result = await chat_service.generate_answer(
        query=sample_user_query,
        context_chunks=[],
        chat_history=None
    )

    assert result["answer"] == "I don't have specific information..."

    # Verify system prompt mentions no context
    call_args = mock_openai_client.chat.completions.create.call_args
    system_prompt = call_args.kwargs["messages"][0]["content"]
    assert "No relevant context found" in system_prompt


@pytest.mark.asyncio
async def test_build_context(
    chat_service: ChatService,
    sample_context_chunks
):
    """Test context building from chunks."""
    context = chat_service._build_context(sample_context_chunks)

    # Verify context includes all chunks
    assert "ROS 2 is the next generation" in context
    assert "Key improvements include DDS" in context

    # Verify source citations
    assert "Source 1" in context
    assert "Week 3" in context
    assert "ROS 2 Fundamentals" in context


@pytest.mark.asyncio
async def test_build_context_empty(chat_service: ChatService):
    """Test context building with no chunks."""
    context = chat_service._build_context([])

    assert "No relevant context found" in context


@pytest.mark.asyncio
async def test_create_system_prompt(chat_service: ChatService):
    """Test system prompt creation."""
    context = "Some textbook content here..."
    prompt = chat_service._create_system_prompt(context)

    # Verify prompt structure
    assert "teaching assistant" in prompt
    assert "Physical AI course" in prompt
    assert context in prompt
    assert "Use the provided context" in prompt


@pytest.mark.asyncio
async def test_generate_answer_model_parameters(
    chat_service: ChatService,
    mock_openai_client,
    sample_user_query: str,
    sample_context_chunks
):
    """Test that correct model parameters are used."""
    mock_response = MagicMock()
    mock_response.choices = [MagicMock(message=MagicMock(content="Answer"))]
    mock_response.usage.total_tokens = 100

    mock_openai_client.chat.completions.create = AsyncMock(return_value=mock_response)

    await chat_service.generate_answer(
        query=sample_user_query,
        context_chunks=sample_context_chunks,
        chat_history=None
    )

    # Verify model parameters
    call_args = mock_openai_client.chat.completions.create.call_args
    assert call_args.kwargs["model"] == "gpt-4o-mini"
    assert call_args.kwargs["temperature"] == 0.3
    assert call_args.kwargs["max_tokens"] == 1000


@pytest.mark.asyncio
async def test_generate_answer_error_handling(
    chat_service: ChatService,
    mock_openai_client,
    sample_user_query: str,
    sample_context_chunks
):
    """Test error handling in answer generation."""
    # Mock API error
    mock_openai_client.chat.completions.create = AsyncMock(
        side_effect=Exception("API error")
    )

    with pytest.raises(Exception, match="API error"):
        await chat_service.generate_answer(
            query=sample_user_query,
            context_chunks=sample_context_chunks,
            chat_history=None
        )


@pytest.mark.asyncio
async def test_close(chat_service: ChatService, mock_openai_client):
    """Test closing chat service client."""
    mock_openai_client.close = AsyncMock()

    await chat_service.close()

    mock_openai_client.close.assert_called_once()
