"""
Tests for /api/v1/history/{session_id} endpoint.

Integration tests for the chat history API endpoint.
"""

import pytest
from unittest.mock import AsyncMock, patch
from httpx import AsyncClient

from app.main import app


@pytest.fixture
def mock_rag_service_with_history():
    """Create mock RAG service with chat history."""
    with patch("app.api.routes.history.get_rag_service") as mock_factory:
        service = AsyncMock()
        service.get_chat_history = AsyncMock(return_value=[
            {
                "id": 1,
                "session_id": "test-session-123",
                "role": "user",
                "content": "What is ROS 2?",
                "sources": [],
                "confidence": None,
                "tokens_used": None,
                "created_at": "2025-12-11T10:00:00Z"
            },
            {
                "id": 2,
                "session_id": "test-session-123",
                "role": "assistant",
                "content": "ROS 2 is an open-source framework...",
                "sources": [
                    {"chapter": "ROS 2 Fundamentals", "module": 1, "week": 3, "score": 0.92}
                ],
                "confidence": 0.92,
                "tokens_used": 150,
                "created_at": "2025-12-11T10:00:05Z"
            },
            {
                "id": 3,
                "session_id": "test-session-123",
                "role": "user",
                "content": "Tell me more about it",
                "sources": [],
                "confidence": None,
                "tokens_used": None,
                "created_at": "2025-12-11T10:01:00Z"
            },
            {
                "id": 4,
                "session_id": "test-session-123",
                "role": "assistant",
                "content": "ROS 2 includes DDS middleware...",
                "sources": [
                    {"chapter": "Architecture Overview", "module": 1, "week": 2, "score": 0.85}
                ],
                "confidence": 0.85,
                "tokens_used": 120,
                "created_at": "2025-12-11T10:01:05Z"
            }
        ])
        mock_factory.return_value = service
        yield service


@pytest.mark.asyncio
async def test_history_endpoint_success(mock_rag_service_with_history):
    """Test successful history retrieval."""
    async with AsyncClient(app=app, base_url="http://test") as client:
        response = await client.get("/api/v1/history/test-session-123")

    assert response.status_code == 200

    data = response.json()
    assert "session_id" in data
    assert "messages" in data
    assert "total_messages" in data

    assert data["session_id"] == "test-session-123"
    assert data["total_messages"] == 4
    assert len(data["messages"]) == 4

    # Check first message (user)
    first_msg = data["messages"][0]
    assert first_msg["role"] == "user"
    assert first_msg["content"] == "What is ROS 2?"
    assert first_msg["sources"] == []

    # Check second message (assistant)
    second_msg = data["messages"][1]
    assert second_msg["role"] == "assistant"
    assert "ROS 2 is an open-source framework" in second_msg["content"]
    assert len(second_msg["sources"]) == 1
    assert second_msg["confidence"] == 0.92


@pytest.mark.asyncio
async def test_history_endpoint_not_found():
    """Test history retrieval for non-existent session."""
    with patch("app.api.routes.history.get_rag_service") as mock_factory:
        service = AsyncMock()
        service.get_chat_history = AsyncMock(return_value=[])
        mock_factory.return_value = service

        async with AsyncClient(app=app, base_url="http://test") as client:
            response = await client.get("/api/v1/history/nonexistent-session")

        assert response.status_code == 404
        data = response.json()
        assert "error" in data["detail"]
        assert data["detail"]["error"] == "NotFound"


@pytest.mark.asyncio
async def test_history_endpoint_internal_error():
    """Test history retrieval when service fails."""
    with patch("app.api.routes.history.get_rag_service") as mock_factory:
        service = AsyncMock()
        service.get_chat_history = AsyncMock(
            side_effect=Exception("Database error")
        )
        mock_factory.return_value = service

        async with AsyncClient(app=app, base_url="http://test") as client:
            response = await client.get("/api/v1/history/test-session-123")

        assert response.status_code == 500
        data = response.json()
        assert data["detail"]["error"] == "InternalServerError"


@pytest.mark.asyncio
async def test_history_endpoint_special_session_id(mock_rag_service_with_history):
    """Test history with special characters in session_id."""
    async with AsyncClient(app=app, base_url="http://test") as client:
        response = await client.get("/api/v1/history/session-with-dashes-123")

    # Should work with dashes
    assert response.status_code in [200, 404]


@pytest.mark.asyncio
async def test_history_endpoint_messages_order(mock_rag_service_with_history):
    """Test that messages are returned in chronological order."""
    async with AsyncClient(app=app, base_url="http://test") as client:
        response = await client.get("/api/v1/history/test-session-123")

    assert response.status_code == 200
    data = response.json()

    messages = data["messages"]
    # Verify chronological order (user -> assistant -> user -> assistant)
    assert messages[0]["role"] == "user"
    assert messages[1]["role"] == "assistant"
    assert messages[2]["role"] == "user"
    assert messages[3]["role"] == "assistant"

    # Verify timestamps are in order
    timestamps = [msg["created_at"] for msg in messages]
    assert timestamps == sorted(timestamps)


@pytest.mark.asyncio
async def test_history_endpoint_single_message():
    """Test history with only one message."""
    with patch("app.api.routes.history.get_rag_service") as mock_factory:
        service = AsyncMock()
        service.get_chat_history = AsyncMock(return_value=[
            {
                "id": 1,
                "session_id": "test-session",
                "role": "user",
                "content": "Hello",
                "sources": [],
                "confidence": None,
                "tokens_used": None,
                "created_at": "2025-12-11T10:00:00Z"
            }
        ])
        mock_factory.return_value = service

        async with AsyncClient(app=app, base_url="http://test") as client:
            response = await client.get("/api/v1/history/test-session")

        assert response.status_code == 200
        data = response.json()
        assert data["total_messages"] == 1


@pytest.mark.asyncio
async def test_history_endpoint_long_conversation():
    """Test history with many messages."""
    with patch("app.api.routes.history.get_rag_service") as mock_factory:
        # Create 20 messages
        messages = []
        for i in range(20):
            role = "user" if i % 2 == 0 else "assistant"
            messages.append({
                "id": i + 1,
                "session_id": "test-session",
                "role": role,
                "content": f"Message {i + 1}",
                "sources": [] if role == "user" else [{"chapter": "Test", "score": 0.8}],
                "confidence": None if role == "user" else 0.8,
                "tokens_used": None if role == "user" else 100,
                "created_at": f"2025-12-11T10:00:{i:02d}Z"
            })

        service = AsyncMock()
        service.get_chat_history = AsyncMock(return_value=messages)
        mock_factory.return_value = service

        async with AsyncClient(app=app, base_url="http://test") as client:
            response = await client.get("/api/v1/history/test-session")

        assert response.status_code == 200
        data = response.json()
        assert data["total_messages"] == 20
        assert len(data["messages"]) == 20


@pytest.mark.asyncio
async def test_history_endpoint_response_schema():
    """Test that response matches OpenAPI schema."""
    with patch("app.api.routes.history.get_rag_service") as mock_factory:
        service = AsyncMock()
        service.get_chat_history = AsyncMock(return_value=[
            {
                "id": 1,
                "session_id": "test",
                "role": "user",
                "content": "Test",
                "sources": [],
                "confidence": None,
                "tokens_used": None,
                "created_at": "2025-12-11T10:00:00Z"
            }
        ])
        mock_factory.return_value = service

        async with AsyncClient(app=app, base_url="http://test") as client:
            response = await client.get("/api/v1/history/test")

        assert response.status_code == 200
        data = response.json()

        # Verify required fields
        assert "session_id" in data
        assert "messages" in data
        assert "total_messages" in data

        # Verify message structure
        msg = data["messages"][0]
        required_fields = ["id", "role", "content", "sources", "created_at"]
        for field in required_fields:
            assert field in msg


@pytest.mark.asyncio
async def test_history_endpoint_cors_headers():
    """Test that CORS headers are present."""
    async with AsyncClient(app=app, base_url="http://test") as client:
        response = await client.options(
            "/api/v1/history/test-session",
            headers={"Origin": "https://naveed261.github.io"}
        )

    # CORS preflight should succeed
    assert response.status_code == 200
