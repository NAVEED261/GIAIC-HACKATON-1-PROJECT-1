"""
History endpoint for retrieving chat conversation history.

Handles GET /api/v1/history/{session_id} requests.
"""

from fastapi import APIRouter, Depends, HTTPException, status, Path
from sqlalchemy.ext.asyncio import AsyncSession
from typing import List

from app.models.responses import HistoryResponse, MessageResponse, ErrorResponse, Source
from app.services.rag_service import RAGService
from app.services.embedding_service import EmbeddingService
from app.services.qdrant_service import QdrantService
from app.services.chat_service import ChatService
from app.db.session import get_db
from app.core.logging import get_logger

logger = get_logger(__name__)

router = APIRouter(prefix="/api/v1", tags=["history"])


# Dependency for RAG service
async def get_rag_service() -> RAGService:
    """Create and return RAG service instance."""
    embedding_service = EmbeddingService()
    qdrant_service = QdrantService()
    chat_service = ChatService()

    return RAGService(
        embedding_service=embedding_service,
        qdrant_service=qdrant_service,
        chat_service=chat_service
    )


@router.get(
    "/history/{session_id}",
    response_model=HistoryResponse,
    status_code=status.HTTP_200_OK,
    responses={
        404: {"model": ErrorResponse, "description": "Session not found"},
        500: {"model": ErrorResponse, "description": "Internal server error"}
    },
    summary="Get chat history for a session",
    description="""
    Retrieve the complete chat history for a given session ID.

    Returns all messages (user questions and assistant answers) in chronological order,
    including sources and confidence scores for assistant responses.
    """
)
async def get_history(
    session_id: str = Path(
        ...,
        description="Session identifier to retrieve history for",
        examples=["session-1234567890-abc"]
    ),
    db: AsyncSession = Depends(get_db),
    rag_service: RAGService = Depends(get_rag_service)
) -> HistoryResponse:
    """
    Retrieve chat history for a session.

    Args:
        session_id: Chat session identifier
        db: Database session
        rag_service: RAG service instance

    Returns:
        HistoryResponse with list of messages

    Raises:
        HTTPException: If session not found or retrieval fails
    """
    try:
        logger.info(f"History request received - Session: {session_id}")

        # Retrieve chat history
        messages = await rag_service.get_chat_history(
            session_id=session_id,
            db=db
        )

        if not messages:
            logger.warning(f"No history found for session: {session_id}")
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail={
                    "error": "NotFound",
                    "message": f"No chat history found for session {session_id}"
                }
            )

        # Convert messages to response format
        message_responses = []
        for msg in messages:
            # Convert sources to Source model
            sources = [
                Source(
                    chapter=src.get("chapter", "Unknown"),
                    module=src.get("module"),
                    week=src.get("week"),
                    score=src.get("score", 0.0)
                )
                for src in msg.get("sources", [])
            ]

            message_responses.append(
                MessageResponse(
                    id=msg["id"],
                    role=msg["role"],
                    content=msg["content"],
                    sources=sources,
                    confidence=msg.get("confidence"),
                    tokens_used=msg.get("tokens_used"),
                    created_at=msg["created_at"]
                )
            )

        response = HistoryResponse(
            session_id=session_id,
            messages=message_responses,
            total_messages=len(message_responses)
        )

        logger.info(
            f"History retrieved - Session: {session_id}, "
            f"Messages: {len(message_responses)}"
        )

        return response

    except HTTPException:
        # Re-raise HTTP exceptions
        raise

    except Exception as e:
        # Internal errors
        logger.error(f"History endpoint error: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "error": "InternalServerError",
                "message": "Failed to retrieve chat history. Please try again."
            }
        )
