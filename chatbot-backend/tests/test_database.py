"""
Tests for database models and operations.

Tests ChatSession and ChatMessage models with SQLAlchemy.
"""

import pytest
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession
from app.db.models import ChatSession, ChatMessage
from datetime import datetime


@pytest.mark.asyncio
async def test_create_chat_session(test_db: AsyncSession, sample_session_id: str):
    """Test creating a new chat session."""
    session = ChatSession(session_id=sample_session_id)

    test_db.add(session)
    await test_db.commit()
    await test_db.refresh(session)

    assert session.id is not None
    assert session.session_id == sample_session_id
    assert session.created_at is not None
    assert session.last_activity is not None
    assert session.metadata_ == {}


@pytest.mark.asyncio
async def test_chat_session_unique_constraint(test_db: AsyncSession, sample_session_id: str):
    """Test that session_id must be unique."""
    session1 = ChatSession(session_id=sample_session_id)
    session2 = ChatSession(session_id=sample_session_id)

    test_db.add(session1)
    await test_db.commit()

    test_db.add(session2)

    with pytest.raises(Exception):  # Should raise IntegrityError
        await test_db.commit()


@pytest.mark.asyncio
async def test_create_chat_message(test_db: AsyncSession, sample_session_id: str):
    """Test creating a chat message linked to a session."""
    # Create session first
    session = ChatSession(session_id=sample_session_id)
    test_db.add(session)
    await test_db.commit()

    # Create message
    message = ChatMessage(
        session_id=sample_session_id,
        role="user",
        content="What is ROS 2?",
        sources=[],
        confidence=None,
        tokens_used=None
    )

    test_db.add(message)
    await test_db.commit()
    await test_db.refresh(message)

    assert message.id is not None
    assert message.session_id == sample_session_id
    assert message.role == "user"
    assert message.content == "What is ROS 2?"
    assert message.sources == []
    assert message.created_at is not None


@pytest.mark.asyncio
async def test_chat_message_with_sources(test_db: AsyncSession, sample_session_id: str):
    """Test creating an assistant message with sources and confidence."""
    # Create session
    session = ChatSession(session_id=sample_session_id)
    test_db.add(session)
    await test_db.commit()

    # Create assistant message with sources
    sources = [
        {"chapter": "ROS 2 Fundamentals", "module": 1, "week": 3, "score": 0.92},
        {"chapter": "Architecture Overview", "module": 1, "week": 2, "score": 0.85}
    ]

    message = ChatMessage(
        session_id=sample_session_id,
        role="assistant",
        content="ROS 2 is an open-source framework...",
        sources=sources,
        confidence=0.92,
        tokens_used=150
    )

    test_db.add(message)
    await test_db.commit()
    await test_db.refresh(message)

    assert message.role == "assistant"
    assert len(message.sources) == 2
    assert message.sources[0]["chapter"] == "ROS 2 Fundamentals"
    assert message.confidence == 0.92
    assert message.tokens_used == 150


@pytest.mark.asyncio
async def test_session_messages_relationship(test_db: AsyncSession, sample_session_id: str):
    """Test the relationship between session and messages."""
    # Create session
    session = ChatSession(session_id=sample_session_id)
    test_db.add(session)
    await test_db.commit()

    # Create multiple messages
    messages_data = [
        {"role": "user", "content": "What is ROS 2?"},
        {"role": "assistant", "content": "ROS 2 is..."},
        {"role": "user", "content": "Tell me more"}
    ]

    for msg_data in messages_data:
        message = ChatMessage(session_id=sample_session_id, **msg_data)
        test_db.add(message)

    await test_db.commit()

    # Query session with messages
    result = await test_db.execute(
        select(ChatSession).where(ChatSession.session_id == sample_session_id)
    )
    retrieved_session = result.scalar_one()

    assert len(retrieved_session.messages) == 3
    assert retrieved_session.messages[0].role == "user"
    assert retrieved_session.messages[1].role == "assistant"


@pytest.mark.asyncio
async def test_cascade_delete(test_db: AsyncSession, sample_session_id: str):
    """Test that deleting a session cascades to messages."""
    # Create session with messages
    session = ChatSession(session_id=sample_session_id)
    test_db.add(session)
    await test_db.commit()

    message = ChatMessage(
        session_id=sample_session_id,
        role="user",
        content="Test message"
    )
    test_db.add(message)
    await test_db.commit()

    # Delete session
    await test_db.delete(session)
    await test_db.commit()

    # Verify messages are also deleted
    result = await test_db.execute(
        select(ChatMessage).where(ChatMessage.session_id == sample_session_id)
    )
    messages = result.scalars().all()

    assert len(messages) == 0


@pytest.mark.asyncio
async def test_message_to_dict(test_db: AsyncSession, sample_session_id: str):
    """Test the to_dict method of ChatMessage."""
    session = ChatSession(session_id=sample_session_id)
    test_db.add(session)
    await test_db.commit()

    message = ChatMessage(
        session_id=sample_session_id,
        role="user",
        content="Test content",
        sources=[{"chapter": "Test", "score": 0.9}],
        confidence=0.9,
        tokens_used=50
    )
    test_db.add(message)
    await test_db.commit()
    await test_db.refresh(message)

    msg_dict = message.to_dict()

    assert msg_dict["session_id"] == sample_session_id
    assert msg_dict["role"] == "user"
    assert msg_dict["content"] == "Test content"
    assert len(msg_dict["sources"]) == 1
    assert msg_dict["confidence"] == 0.9
    assert msg_dict["tokens_used"] == 50
    assert "created_at" in msg_dict
