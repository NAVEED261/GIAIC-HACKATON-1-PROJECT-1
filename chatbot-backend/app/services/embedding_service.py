"""
OpenAI Embedding Service for generating text embeddings.

Provides async methods to generate embeddings using OpenAI's text-embedding-3-small model.
"""

from typing import List, Dict, Any
from openai import AsyncOpenAI
from ..core.config import settings
from ..core.logging import get_logger

logger = get_logger(__name__)


class EmbeddingService:
    """Service for generating text embeddings using OpenAI API."""

    def __init__(self, api_key: str = settings.OPENAI_API_KEY):
        """
        Initialize embedding service with OpenAI API key.

        Args:
            api_key: OpenAI API key (defaults to settings.OPENAI_API_KEY)
        """
        self.client = AsyncOpenAI(api_key=api_key)
        self.model = settings.OPENAI_EMBEDDING_MODEL
        logger.info(f"Initialized EmbeddingService with model: {self.model}")

    async def embed_text(self, text: str) -> List[float]:
        """
        Generate embedding vector for a single text.

        Args:
            text: Input text to embed

        Returns:
            List of floats representing the embedding vector (1536 dimensions)

        Raises:
            Exception: If embedding generation fails
        """
        try:
            response = await self.client.embeddings.create(
                model=self.model,
                input=text
            )

            embedding = response.data[0].embedding
            logger.debug(f"Generated embedding for text (length: {len(text)} chars)")

            return embedding

        except Exception as e:
            logger.error(f"Failed to generate embedding: {e}")
            raise

    async def embed_batch(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for multiple texts in a single API call.

        Args:
            texts: List of input texts to embed

        Returns:
            List of embedding vectors, one for each input text

        Raises:
            Exception: If batch embedding generation fails
        """
        try:
            response = await self.client.embeddings.create(
                model=self.model,
                input=texts
            )

            embeddings = [item.embedding for item in response.data]
            logger.debug(f"Generated {len(embeddings)} embeddings in batch")

            return embeddings

        except Exception as e:
            logger.error(f"Failed to generate batch embeddings: {e}")
            raise

    async def embed_query(self, query: str) -> List[float]:
        """
        Generate embedding for a user query.

        This is a convenience method that's semantically equivalent to embed_text,
        but makes the code more readable when embedding user queries for search.

        Args:
            query: User query text

        Returns:
            Embedding vector for the query
        """
        logger.info(f"Embedding user query: '{query[:100]}...'")
        return await self.embed_text(query)

    def get_embedding_dimension(self) -> int:
        """
        Get the dimension of embeddings produced by this service.

        Returns:
            int: Embedding dimension (1536 for text-embedding-3-small)
        """
        return settings.QDRANT_VECTOR_SIZE

    async def close(self) -> None:
        """Close the OpenAI client connection."""
        await self.client.close()
        logger.info("EmbeddingService client closed")
