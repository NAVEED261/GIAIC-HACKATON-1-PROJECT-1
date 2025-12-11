"""
Tests for /api/v1/chat endpoint.

Integration tests for the chat API endpoint.
"""

import pytest
from unittest.mock import AsyncMock, patch
from fastapi.testclient import TestClient
from httpx import AsyncClient

from app.main import app


@pytest.fixture
def mock_rag_service():
    """Create mock RAG service."""
    with patch("app.api.routes.chat.get_rag_service") as mock_factory:
        service = AsyncMock()
        service.generate_answer = AsyncMock(return_value={
            "answer": "ROS 2 is an open-source framework for robot development...",
            "sources": [
                {"chapter": "ROS 2 Fundamentals", "module": 1, "week": 3, "score": 0.92},
                {"chapter": "Architecture Overview", "module": 1, "week": 2, "score": 0.85}
            ],
            "session_id": "test-session-123",
            "confidence": 0.92
        })
        mock_factory.return_value = service
        yield service


@pytest.mark.asyncio
async def test_chat_endpoint_success(mock_rag_service):
    """Test successful chat request."""
    async with AsyncClient(app=app, base_url="http://test") as client:
        response = await client.post(
            "/api/v1/chat",
            json={
                "query": "What is ROS 2?",
                "session_id": "test-session-123"
            }
        )

    assert response.status_code == 200

    data = response.json()
    assert "answer" in data
    assert "sources" in data
    assert "session_id" in data
    assert "confidence" in data

    assert data["answer"] == "ROS 2 is an open-source framework for robot development..."
    assert len(data["sources"]) == 2
    assert data["sources"][0]["chapter"] == "ROS 2 Fundamentals"
    assert data["confidence"] == 0.92


@pytest.mark.asyncio
async def test_chat_endpoint_empty_query(mock_rag_service):
    """Test chat request with empty query."""
    async with AsyncClient(app=app, base_url="http://test") as client:
        response = await client.post(
            "/api/v1/chat",
            json={
                "query": "",
                "session_id": "test-session-123"
            }
        )

    assert response.status_code == 422  # Validation error


@pytest.mark.asyncio
async def test_chat_endpoint_whitespace_query(mock_rag_service):
    """Test chat request with whitespace-only query."""
    async with AsyncClient(app=app, base_url="http://test") as client:
        response = await client.post(
            "/api/v1/chat",
            json={
                "query": "   ",
                "session_id": "test-session-123"
            }
        )

    assert response.status_code == 422  # Validation error


@pytest.mark.asyncio
async def test_chat_endpoint_missing_session_id(mock_rag_service):
    """Test chat request without session_id."""
    async with AsyncClient(app=app, base_url="http://test") as client:
        response = await client.post(
            "/api/v1/chat",
            json={
                "query": "What is ROS 2?"
            }
        )

    assert response.status_code == 422  # Validation error


@pytest.mark.asyncio
async def test_chat_endpoint_long_query(mock_rag_service):
    """Test chat request with very long query."""
    long_query = "A" * 3000  # Exceeds max_length=2000

    async with AsyncClient(app=app, base_url="http://test") as client:
        response = await client.post(
            "/api/v1/chat",
            json={
                "query": long_query,
                "session_id": "test-session-123"
            }
        )

    assert response.status_code == 422  # Validation error


@pytest.mark.asyncio
async def test_chat_endpoint_internal_error():
    """Test chat request when RAG service fails."""
    with patch("app.api.routes.chat.get_rag_service") as mock_factory:
        service = AsyncMock()
        service.generate_answer = AsyncMock(
            side_effect=Exception("Internal error")
        )
        mock_factory.return_value = service

        async with AsyncClient(app=app, base_url="http://test") as client:
            response = await client.post(
                "/api/v1/chat",
                json={
                    "query": "What is ROS 2?",
                    "session_id": "test-session-123"
                }
            )

        assert response.status_code == 500
        data = response.json()
        assert "error" in data["detail"]
        assert data["detail"]["error"] == "InternalServerError"


@pytest.mark.asyncio
async def test_chat_endpoint_cors_headers():
    """Test that CORS headers are present in response."""
    async with AsyncClient(app=app, base_url="http://test") as client:
        response = await client.options(
            "/api/v1/chat",
            headers={"Origin": "https://naveed261.github.io"}
        )

    # CORS preflight should succeed
    assert response.status_code == 200


@pytest.mark.asyncio
async def test_chat_endpoint_multiple_sources(mock_rag_service):
    """Test chat response with multiple source citations."""
    # Update mock to return 5 sources
    mock_rag_service.generate_answer = AsyncMock(return_value={
        "answer": "Detailed answer...",
        "sources": [
            {"chapter": f"Chapter {i}", "module": 1, "week": i, "score": 0.9 - (i * 0.05)}
            for i in range(1, 6)
        ],
        "session_id": "test-session-123",
        "confidence": 0.9
    })

    async with AsyncClient(app=app, base_url="http://test") as client:
        response = await client.post(
            "/api/v1/chat",
            json={
                "query": "Complex question requiring multiple sources",
                "session_id": "test-session-123"
            }
        )

    assert response.status_code == 200
    data = response.json()
    assert len(data["sources"]) == 5


@pytest.mark.asyncio
async def test_chat_endpoint_no_sources(mock_rag_service):
    """Test chat response when no relevant sources found."""
    mock_rag_service.generate_answer = AsyncMock(return_value={
        "answer": "I don't have specific information about that...",
        "sources": [],
        "session_id": "test-session-123",
        "confidence": 0.0
    })

    async with AsyncClient(app=app, base_url="http://test") as client:
        response = await client.post(
            "/api/v1/chat",
            json={
                "query": "Unrelated question",
                "session_id": "test-session-123"
            }
        )

    assert response.status_code == 200
    data = response.json()
    assert len(data["sources"]) == 0
    assert data["confidence"] == 0.0


@pytest.mark.asyncio
async def test_chat_endpoint_special_characters(mock_rag_service):
    """Test chat with special characters in query."""
    async with AsyncClient(app=app, base_url="http://test") as client:
        response = await client.post(
            "/api/v1/chat",
            json={
                "query": "What is ROS 2? How does it differ from ROS 1?",
                "session_id": "test-session-123"
            }
        )

    assert response.status_code == 200


@pytest.mark.asyncio
async def test_chat_endpoint_unicode_query(mock_rag_service):
    """Test chat with unicode characters in query."""
    async with AsyncClient(app=app, base_url="http://test") as client:
        response = await client.post(
            "/api/v1/chat",
            json={
                "query": "What is the robot's trajectory? 🤖",
                "session_id": "test-session-123"
            }
        )

    assert response.status_code == 200


@pytest.mark.asyncio
async def test_chat_endpoint_response_schema():
    """Test that response matches OpenAPI schema."""
    with patch("app.api.routes.chat.get_rag_service") as mock_factory:
        service = AsyncMock()
        service.generate_answer = AsyncMock(return_value={
            "answer": "Test answer",
            "sources": [{"chapter": "Test", "module": 1, "week": 1, "score": 0.8}],
            "session_id": "test-session",
            "confidence": 0.8
        })
        mock_factory.return_value = service

        async with AsyncClient(app=app, base_url="http://test") as client:
            response = await client.post(
                "/api/v1/chat",
                json={
                    "query": "Test query",
                    "session_id": "test-session"
                }
            )

        assert response.status_code == 200
        data = response.json()

        # Verify all required fields present
        required_fields = ["answer", "sources", "session_id", "confidence"]
        for field in required_fields:
            assert field in data

        # Verify sources structure
        assert isinstance(data["sources"], list)
        if data["sources"]:
            source = data["sources"][0]
            assert "chapter" in source
            assert "score" in source
