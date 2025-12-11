"""
Tests for Qdrant vector database service.

Tests Qdrant collection creation, upsert, and search operations.
"""

import pytest
from unittest.mock import AsyncMock, MagicMock, patch
from app.services.qdrant_service import QdrantService
from qdrant_client.models import Distance, VectorParams, ScoredPoint


@pytest.fixture
def mock_qdrant_client():
    """Create a mock Qdrant client."""
    with patch("app.services.qdrant_service.QdrantClient") as mock_client:
        yield mock_client.return_value


@pytest.fixture
def qdrant_service(mock_qdrant_client):
    """Create QdrantService instance with mocked client."""
    service = QdrantService()
    service.client = mock_qdrant_client
    return service


@pytest.mark.asyncio
async def test_create_collection_new(qdrant_service: QdrantService, mock_qdrant_client):
    """Test creating a new collection when it doesn't exist."""
    # Mock get_collections to return empty list
    mock_collections = MagicMock()
    mock_collections.collections = []
    mock_qdrant_client.get_collections.return_value = mock_collections

    await qdrant_service.create_collection()

    # Verify create_collection was called
    mock_qdrant_client.create_collection.assert_called_once()
    call_args = mock_qdrant_client.create_collection.call_args

    assert call_args.kwargs["collection_name"] == "textbook_chunks"
    assert isinstance(call_args.kwargs["vectors_config"], VectorParams)


@pytest.mark.asyncio
async def test_create_collection_existing(qdrant_service: QdrantService, mock_qdrant_client):
    """Test that existing collection is not recreated."""
    # Mock get_collections to return existing collection
    mock_collection = MagicMock()
    mock_collection.name = "textbook_chunks"

    mock_collections = MagicMock()
    mock_collections.collections = [mock_collection]
    mock_qdrant_client.get_collections.return_value = mock_collections

    await qdrant_service.create_collection()

    # Verify create_collection was NOT called
    mock_qdrant_client.create_collection.assert_not_called()


@pytest.mark.asyncio
async def test_upsert_points(
    qdrant_service: QdrantService,
    mock_qdrant_client,
    sample_embedding,
    sample_qdrant_payload
):
    """Test upserting points to collection."""
    embeddings = [sample_embedding, sample_embedding]
    payloads = [sample_qdrant_payload, sample_qdrant_payload]

    await qdrant_service.upsert_points(embeddings, payloads)

    # Verify upsert was called
    mock_qdrant_client.upsert.assert_called_once()
    call_args = mock_qdrant_client.upsert.call_args

    assert call_args.kwargs["collection_name"] == "textbook_chunks"
    assert len(call_args.kwargs["points"]) == 2


@pytest.mark.asyncio
async def test_upsert_points_mismatched_lengths(qdrant_service: QdrantService, sample_embedding):
    """Test that mismatched embeddings and payloads raise error."""
    embeddings = [sample_embedding]
    payloads = [{}, {}]  # Different length

    with pytest.raises(ValueError, match="must match"):
        await qdrant_service.upsert_points(embeddings, payloads)


@pytest.mark.asyncio
async def test_search_basic(
    qdrant_service: QdrantService,
    mock_qdrant_client,
    sample_embedding
):
    """Test basic semantic search."""
    # Mock search results
    mock_result = MagicMock(spec=ScoredPoint)
    mock_result.id = "test-id-1"
    mock_result.score = 0.95
    mock_result.payload = {
        "text": "ROS 2 is...",
        "chapter": "ROS 2 Fundamentals",
        "module": 1,
        "week": 3
    }

    mock_qdrant_client.search.return_value = [mock_result]

    results = await qdrant_service.search(
        query_vector=sample_embedding,
        limit=5,
        score_threshold=0.7
    )

    assert len(results) == 1
    assert results[0]["id"] == "test-id-1"
    assert results[0]["score"] == 0.95
    assert results[0]["payload"]["chapter"] == "ROS 2 Fundamentals"

    # Verify search was called with correct parameters
    mock_qdrant_client.search.assert_called_once()
    call_args = mock_qdrant_client.search.call_args

    assert call_args.kwargs["collection_name"] == "textbook_chunks"
    assert call_args.kwargs["limit"] == 5
    assert call_args.kwargs["score_threshold"] == 0.7


@pytest.mark.asyncio
async def test_search_with_filters(
    qdrant_service: QdrantService,
    mock_qdrant_client,
    sample_embedding
):
    """Test search with metadata filters."""
    mock_qdrant_client.search.return_value = []

    filters = {"module": 1, "week": 3}

    await qdrant_service.search(
        query_vector=sample_embedding,
        filters=filters
    )

    # Verify filter was applied
    call_args = mock_qdrant_client.search.call_args
    assert call_args.kwargs["query_filter"] is not None


@pytest.mark.asyncio
async def test_get_collection_info(qdrant_service: QdrantService, mock_qdrant_client):
    """Test getting collection information."""
    # Mock collection info
    mock_info = MagicMock()
    mock_info.vectors_count = 1000
    mock_info.points_count = 1000
    mock_info.status = "green"

    mock_qdrant_client.get_collection.return_value = mock_info

    info = qdrant_service.get_collection_info()

    assert info["name"] == "textbook_chunks"
    assert info["vectors_count"] == 1000
    assert info["points_count"] == 1000
    assert info["status"] == "green"


@pytest.mark.asyncio
async def test_delete_collection(qdrant_service: QdrantService, mock_qdrant_client):
    """Test deleting a collection."""
    await qdrant_service.delete_collection()

    mock_qdrant_client.delete_collection.assert_called_once_with(
        collection_name="textbook_chunks"
    )


def test_close(qdrant_service: QdrantService, mock_qdrant_client):
    """Test closing Qdrant client."""
    qdrant_service.close()

    mock_qdrant_client.close.assert_called_once()
