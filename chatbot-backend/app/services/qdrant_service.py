"""
Qdrant Vector Database Service for semantic search.

Provides methods to create collections, store embeddings, and perform similarity search.
"""

from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.models import (
    Distance,
    VectorParams,
    PointStruct,
    SearchRequest,
    Filter,
    FieldCondition,
    MatchValue
)
import uuid
from ..core.config import settings
from ..core.logging import get_logger

logger = get_logger(__name__)


class QdrantService:
    """Service for interacting with Qdrant vector database."""

    def __init__(
        self,
        url: str = settings.QDRANT_URL,
        api_key: str = settings.QDRANT_API_KEY,
        collection_name: str = settings.QDRANT_COLLECTION_NAME
    ):
        """
        Initialize Qdrant service with connection parameters.

        Args:
            url: Qdrant Cloud URL
            api_key: Qdrant API key
            collection_name: Name of the collection to use
        """
        self.client = QdrantClient(url=url, api_key=api_key)
        self.collection_name = collection_name
        self.vector_size = settings.QDRANT_VECTOR_SIZE

        logger.info(f"Initialized QdrantService with collection: {collection_name}")

    async def create_collection(self) -> None:
        """
        Create a new Qdrant collection with cosine similarity.

        This should be called once during initial setup.
        """
        try:
            # Check if collection already exists
            collections = self.client.get_collections().collections
            if any(col.name == self.collection_name for col in collections):
                logger.info(f"Collection '{self.collection_name}' already exists")
                return

            # Create new collection
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=self.vector_size,
                    distance=Distance.COSINE
                )
            )

            logger.info(f"Created collection '{self.collection_name}' with vector size {self.vector_size}")

        except Exception as e:
            logger.error(f"Failed to create collection: {e}")
            raise

    async def upsert_points(
        self,
        embeddings: List[List[float]],
        payloads: List[Dict[str, Any]]
    ) -> None:
        """
        Insert or update vectors in the collection.

        Args:
            embeddings: List of embedding vectors
            payloads: List of metadata dictionaries (chapter, module, week, text, file_path)

        Raises:
            ValueError: If embeddings and payloads lengths don't match
        """
        if len(embeddings) != len(payloads):
            raise ValueError(
                f"Embeddings count ({len(embeddings)}) must match payloads count ({len(payloads)})"
            )

        try:
            points = [
                PointStruct(
                    id=str(uuid.uuid4()),
                    vector=embedding,
                    payload=payload
                )
                for embedding, payload in zip(embeddings, payloads)
            ]

            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )

            logger.info(f"Upserted {len(points)} points to collection '{self.collection_name}'")

        except Exception as e:
            logger.error(f"Failed to upsert points: {e}")
            raise

    async def search(
        self,
        query_vector: List[float],
        limit: int = settings.QDRANT_SEARCH_LIMIT,
        score_threshold: float = settings.QDRANT_SCORE_THRESHOLD,
        filters: Optional[Dict[str, Any]] = None
    ) -> List[Dict[str, Any]]:
        """
        Perform semantic search using query vector.

        Args:
            query_vector: Query embedding vector
            limit: Maximum number of results to return
            score_threshold: Minimum similarity score (0.0 to 1.0)
            filters: Optional metadata filters (e.g., {"module": 1, "week": 3})

        Returns:
            List of search results with payload and score
        """
        try:
            # Build filter if provided
            query_filter = None
            if filters:
                conditions = [
                    FieldCondition(
                        key=key,
                        match=MatchValue(value=value)
                    )
                    for key, value in filters.items()
                ]
                query_filter = Filter(must=conditions)

            # Perform search
            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=limit,
                score_threshold=score_threshold,
                query_filter=query_filter
            )

            # Format results
            formatted_results = [
                {
                    "id": result.id,
                    "score": result.score,
                    "payload": result.payload
                }
                for result in results
            ]

            logger.info(
                f"Found {len(formatted_results)} results "
                f"(threshold: {score_threshold}, limit: {limit})"
            )

            return formatted_results

        except Exception as e:
            logger.error(f"Search failed: {e}")
            raise

    async def delete_collection(self) -> None:
        """Delete the entire collection (use with caution!)."""
        try:
            self.client.delete_collection(collection_name=self.collection_name)
            logger.warning(f"Deleted collection '{self.collection_name}'")

        except Exception as e:
            logger.error(f"Failed to delete collection: {e}")
            raise

    def get_collection_info(self) -> Dict[str, Any]:
        """
        Get information about the collection.

        Returns:
            Dictionary with collection stats (vectors_count, status, etc.)
        """
        try:
            info = self.client.get_collection(collection_name=self.collection_name)
            return {
                "name": self.collection_name,
                "vectors_count": info.vectors_count,
                "points_count": info.points_count,
                "status": info.status,
                "vector_size": self.vector_size
            }

        except Exception as e:
            logger.error(f"Failed to get collection info: {e}")
            raise

    def close(self) -> None:
        """Close Qdrant client connection."""
        self.client.close()
        logger.info("QdrantService client closed")
