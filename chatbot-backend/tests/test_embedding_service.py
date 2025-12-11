"""
Tests for OpenAI embedding service.

Tests embedding generation for single texts, batches, and queries.
"""

import pytest
from unittest.mock import AsyncMock, MagicMock, patch
from app.services.embedding_service import EmbeddingService


@pytest.fixture
def mock_openai_client():
    """Create a mock OpenAI client."""
    with patch("app.services.embedding_service.AsyncOpenAI") as mock_client:
        yield mock_client.return_value


@pytest.fixture
def embedding_service(mock_openai_client):
    """Create EmbeddingService instance with mocked client."""
    service = EmbeddingService()
    service.client = mock_openai_client
    return service


@pytest.mark.asyncio
async def test_embed_text(
    embedding_service: EmbeddingService,
    mock_openai_client,
    sample_user_query,
    sample_embedding
):
    """Test generating embedding for a single text."""
    # Mock OpenAI response
    mock_response = MagicMock()
    mock_response.data = [MagicMock(embedding=sample_embedding)]

    mock_openai_client.embeddings.create = AsyncMock(return_value=mock_response)

    result = await embedding_service.embed_text(sample_user_query)

    assert result == sample_embedding
    assert len(result) == 1536

    # Verify API was called correctly
    mock_openai_client.embeddings.create.assert_called_once_with(
        model="text-embedding-3-small",
        input=sample_user_query
    )


@pytest.mark.asyncio
async def test_embed_batch(
    embedding_service: EmbeddingService,
    mock_openai_client,
    sample_embedding
):
    """Test generating embeddings for multiple texts."""
    texts = [
        "What is ROS 2?",
        "Explain digital twins",
        "How does NVIDIA Isaac Sim work?"
    ]

    # Mock batch response
    mock_response = MagicMock()
    mock_response.data = [
        MagicMock(embedding=sample_embedding),
        MagicMock(embedding=sample_embedding),
        MagicMock(embedding=sample_embedding)
    ]

    mock_openai_client.embeddings.create = AsyncMock(return_value=mock_response)

    results = await embedding_service.embed_batch(texts)

    assert len(results) == 3
    assert all(len(emb) == 1536 for emb in results)

    # Verify batch API call
    mock_openai_client.embeddings.create.assert_called_once_with(
        model="text-embedding-3-small",
        input=texts
    )


@pytest.mark.asyncio
async def test_embed_query(
    embedding_service: EmbeddingService,
    mock_openai_client,
    sample_user_query,
    sample_embedding
):
    """Test embedding a user query (convenience method)."""
    mock_response = MagicMock()
    mock_response.data = [MagicMock(embedding=sample_embedding)]

    mock_openai_client.embeddings.create = AsyncMock(return_value=mock_response)

    result = await embedding_service.embed_query(sample_user_query)

    assert result == sample_embedding
    mock_openai_client.embeddings.create.assert_called_once()


@pytest.mark.asyncio
async def test_embed_text_error(
    embedding_service: EmbeddingService,
    mock_openai_client,
    sample_user_query
):
    """Test that embedding errors are properly raised."""
    # Mock API error
    mock_openai_client.embeddings.create = AsyncMock(
        side_effect=Exception("API rate limit exceeded")
    )

    with pytest.raises(Exception, match="API rate limit exceeded"):
        await embedding_service.embed_text(sample_user_query)


@pytest.mark.asyncio
async def test_embed_batch_error(
    embedding_service: EmbeddingService,
    mock_openai_client
):
    """Test that batch embedding errors are properly raised."""
    texts = ["Text 1", "Text 2"]

    mock_openai_client.embeddings.create = AsyncMock(
        side_effect=Exception("Invalid API key")
    )

    with pytest.raises(Exception, match="Invalid API key"):
        await embedding_service.embed_batch(texts)


def test_get_embedding_dimension(embedding_service: EmbeddingService):
    """Test getting embedding dimension."""
    dimension = embedding_service.get_embedding_dimension()

    assert dimension == 1536
    assert isinstance(dimension, int)


@pytest.mark.asyncio
async def test_close(embedding_service: EmbeddingService, mock_openai_client):
    """Test closing OpenAI client."""
    mock_openai_client.close = AsyncMock()

    await embedding_service.close()

    mock_openai_client.close.assert_called_once()


@pytest.mark.asyncio
async def test_embed_empty_text(
    embedding_service: EmbeddingService,
    mock_openai_client,
    sample_embedding
):
    """Test embedding empty or whitespace-only text."""
    # Mock response
    mock_response = MagicMock()
    mock_response.data = [MagicMock(embedding=sample_embedding)]

    mock_openai_client.embeddings.create = AsyncMock(return_value=mock_response)

    # Test with empty string
    result = await embedding_service.embed_text("")

    assert result == sample_embedding

    # Test with whitespace
    result = await embedding_service.embed_text("   ")

    assert result == sample_embedding
