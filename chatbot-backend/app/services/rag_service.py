"""
RAG (Retrieval-Augmented Generation) Service.

Orchestrates the complete RAG pipeline: embedding generation, vector search,
and chat completion to answer user queries.
"""

from typing import List, Dict, Any, Optional
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select
from .embedding_service import EmbeddingService
from .qdrant_service import QdrantService
from .chat_service import ChatService
from ..db.models import ChatSession, ChatMessage
from ..core.config import settings
from ..core.logging import get_logger

logger = get_logger(__name__)


class RAGService:
    """
    RAG Service orchestrating the complete question-answering pipeline.

    Flow:
    1. Generate embedding for user query
    2. Search vector database for relevant chunks
    3. Generate answer using retrieved context
    4. Store conversation in database
    """

    def __init__(
        self,
        embedding_service: EmbeddingService,
        qdrant_service: QdrantService,
        chat_service: ChatService
    ):
        """
        Initialize RAG service with dependencies.

        Args:
            embedding_service: Service for generating embeddings
            qdrant_service: Service for vector search
            chat_service: Service for chat completions
        """
        self.embedding_service = embedding_service
        self.qdrant_service = qdrant_service
        self.chat_service = chat_service
        logger.info("Initialized RAGService")

    async def generate_answer(
        self,
        query: str,
        session_id: str,
        db: AsyncSession
    ) -> Dict[str, Any]:
        """
        Generate an answer to a user query using RAG pipeline.

        Args:
            query: User's question
            session_id: Chat session identifier
            db: Database session

        Returns:
            Dictionary containing:
                - answer: Generated response
                - sources: List of source chunks with metadata
                - session_id: Session identifier
                - confidence: Confidence score (max similarity)
        """
        try:
            logger.info(f"Processing query for session {session_id}: '{query[:100]}...'")

            # Step 1: Try to generate embedding and search for query
            search_results = []
            try:
                query_embedding = await self.embedding_service.embed_query(query)

                # Step 2: Search for relevant chunks
                search_results = await self.qdrant_service.search(
                    query_vector=query_embedding,
                    limit=settings.MAX_CONTEXT_CHUNKS,
                    score_threshold=settings.MIN_CONFIDENCE_THRESHOLD
                )
            except Exception as search_error:
                logger.warning(f"Vector search failed (fallback mode): {search_error}")
                # Continue with empty search results (fallback mode)

            logger.info(f"Found {len(search_results)} relevant chunks")

            # Step 3: Get chat history for context (with fallback)
            chat_history = []
            try:
                chat_history = await self._get_chat_history(session_id, db)
            except Exception as db_error:
                logger.warning(f"Failed to retrieve chat history: {db_error}")

            # Step 4: Generate answer using context
            answer_data = await self.chat_service.generate_answer(
                query=query,
                context_chunks=search_results,
                chat_history=chat_history
            )

            # Add fallback notice if no context was found
            if not search_results:
                answer_data["answer"] = (
                    f"{answer_data['answer']}\n\n"
                    "ℹ️ Note: I'm currently responding without access to specific course documents. "
                    "My responses are based on general AI knowledge about robotics and Physical AI."
                )

            # Step 5: Calculate confidence (max similarity score)
            confidence = max([chunk["score"] for chunk in search_results]) if search_results else 0.5

            # Step 6: Format sources
            sources = self._format_sources(search_results)

            # Step 7: Store conversation in database (with fallback)
            try:
                await self._store_conversation(
                    session_id=session_id,
                    query=query,
                    answer=answer_data["answer"],
                    sources=sources,
                    confidence=confidence,
                    tokens_used=answer_data["tokens_used"],
                    db=db
                )
            except Exception as store_error:
                logger.warning(f"Failed to store conversation: {store_error}")

            logger.info(
                f"Generated answer (confidence: {confidence:.2f}, "
                f"tokens: {answer_data['tokens_used']}, "
                f"sources: {len(sources)})"
            )

            return {
                "answer": answer_data["answer"],
                "sources": sources,
                "session_id": session_id,
                "confidence": confidence
            }

        except Exception as e:
            logger.error(f"RAG pipeline failed: {e}")
            raise

    async def _get_chat_history(
        self,
        session_id: str,
        db: AsyncSession,
        limit: int = 5
    ) -> List[Dict[str, str]]:
        """
        Retrieve recent chat history for context.

        Args:
            session_id: Chat session identifier
            db: Database session
            limit: Maximum number of messages to retrieve

        Returns:
            List of message dictionaries with role and content
        """
        try:
            result = await db.execute(
                select(ChatMessage)
                .where(ChatMessage.session_id == session_id)
                .order_by(ChatMessage.created_at.desc())
                .limit(limit)
            )

            messages = result.scalars().all()

            # Reverse to get chronological order
            chat_history = [
                {"role": msg.role, "content": msg.content}
                for msg in reversed(messages)
            ]

            return chat_history

        except Exception as e:
            logger.warning(f"Failed to retrieve chat history: {e}")
            return []

    def _format_sources(self, search_results: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Format search results into source citations.

        Args:
            search_results: Raw search results from Qdrant

        Returns:
            List of formatted source dictionaries
        """
        sources = []

        for result in search_results:
            payload = result.get("payload", {})
            sources.append({
                "chapter": payload.get("chapter", "Unknown"),
                "module": payload.get("module"),
                "week": payload.get("week"),
                "score": round(result.get("score", 0.0), 2)
            })

        return sources

    async def _store_conversation(
        self,
        session_id: str,
        query: str,
        answer: str,
        sources: List[Dict[str, Any]],
        confidence: float,
        tokens_used: int,
        db: AsyncSession
    ) -> None:
        """
        Store conversation in database.

        Args:
            session_id: Chat session identifier
            query: User's question
            answer: Generated answer
            sources: Source citations
            confidence: Confidence score
            tokens_used: Tokens consumed
            db: Database session
        """
        try:
            # Ensure session exists
            result = await db.execute(
                select(ChatSession).where(ChatSession.session_id == session_id)
            )
            session = result.scalar_one_or_none()

            if not session:
                # Create new session
                session = ChatSession(session_id=session_id)
                db.add(session)
                await db.flush()
                logger.info(f"Created new chat session: {session_id}")

            # Store user message
            user_message = ChatMessage(
                session_id=session_id,
                role="user",
                content=query,
                sources=[],
                confidence=None,
                tokens_used=None
            )
            db.add(user_message)

            # Store assistant message
            assistant_message = ChatMessage(
                session_id=session_id,
                role="assistant",
                content=answer,
                sources=sources,
                confidence=confidence,
                tokens_used=tokens_used
            )
            db.add(assistant_message)

            await db.commit()

            logger.info(f"Stored conversation for session {session_id}")

        except Exception as e:
            await db.rollback()
            logger.error(f"Failed to store conversation: {e}")
            raise

    async def get_chat_history(
        self,
        session_id: str,
        db: AsyncSession
    ) -> List[Dict[str, Any]]:
        """
        Retrieve full chat history for a session.

        Args:
            session_id: Chat session identifier
            db: Database session

        Returns:
            List of message dictionaries
        """
        try:
            result = await db.execute(
                select(ChatMessage)
                .where(ChatMessage.session_id == session_id)
                .order_by(ChatMessage.created_at.asc())
            )

            messages = result.scalars().all()

            return [msg.to_dict() for msg in messages]

        except Exception as e:
            logger.error(f"Failed to retrieve chat history: {e}")
            raise
