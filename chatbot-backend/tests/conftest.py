"""
Pytest configuration and shared fixtures for tests.
"""

import pytest
import asyncio
from typing import AsyncGenerator
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession, async_sessionmaker
from app.db.models import Base
from app.core.config import settings


@pytest.fixture(scope="session")
def event_loop():
    """Create an event loop for the test session."""
    loop = asyncio.get_event_loop_policy().new_event_loop()
    yield loop
    loop.close()


@pytest.fixture(scope="function")
async def test_db() -> AsyncGenerator[AsyncSession, None]:
    """
    Create a test database session with isolated transactions.

    Each test gets a fresh database session that's rolled back after the test.
    """
    # Use in-memory SQLite for fast tests
    # For Postgres-specific features, you'd use a test Postgres database
    test_engine = create_async_engine(
        "sqlite+aiosqlite:///:memory:",
        echo=False
    )

    # Create all tables
    async with test_engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)

    # Create session factory
    TestSessionLocal = async_sessionmaker(
        test_engine,
        class_=AsyncSession,
        expire_on_commit=False
    )

    # Provide session to test
    async with TestSessionLocal() as session:
        yield session

    # Cleanup
    await test_engine.dispose()


@pytest.fixture
def sample_session_id() -> str:
    """Provide a sample session ID for tests."""
    return "test-session-123"


@pytest.fixture
def sample_user_query() -> str:
    """Provide a sample user query for tests."""
    return "What is ROS 2 and how does it differ from ROS 1?"


@pytest.fixture
def sample_embedding() -> list[float]:
    """Provide a sample embedding vector (1536 dimensions)."""
    # Generate a dummy 1536-dimensional vector for testing
    return [0.1] * 1536


@pytest.fixture
def sample_qdrant_payload() -> dict:
    """Provide sample Qdrant metadata payload."""
    return {
        "text": "ROS 2 is the next generation Robot Operating System...",
        "chapter": "ROS 2 Fundamentals",
        "module": 1,
        "week": 3,
        "file_path": "docs/week-03/ros2-fundamentals.md"
    }
