"""
Chat endpoint for RAG-powered question answering.

Handles POST /api/v1/chat requests for answering user queries.
"""

from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession
from typing import Dict, Any

from app.models.requests import ChatRequest
from app.models.responses import ChatResponse, ErrorResponse
from app.services.rag_service import RAGService
from app.services.embedding_service import EmbeddingService
from app.services.qdrant_service import QdrantService
from app.services.chat_service import ChatService
from app.db.session import get_db
from app.core.logging import get_logger

logger = get_logger(__name__)

router = APIRouter(prefix="/api/v1", tags=["chat"])


# Dependency for RAG service
async def get_rag_service() -> RAGService:
    """
    Create and return RAG service instance.

    This creates new service instances for each request.
    In production, you might want to use dependency injection with singleton services.
    """
    embedding_service = EmbeddingService()
    qdrant_service = QdrantService()
    chat_service = ChatService()

    return RAGService(
        embedding_service=embedding_service,
        qdrant_service=qdrant_service,
        chat_service=chat_service
    )


@router.post(
    "/chat",
    response_model=ChatResponse,
    status_code=status.HTTP_200_OK,
    responses={
        400: {"model": ErrorResponse, "description": "Invalid request"},
        500: {"model": ErrorResponse, "description": "Internal server error"}
    },
    summary="Ask a question about the course",
    description="""
    Submit a question about the Physical AI course content and receive an AI-generated answer
    with source citations from the textbook.

    The endpoint:
    1. Generates embeddings for your query
    2. Searches for relevant textbook content
    3. Uses AI to generate a comprehensive answer
    4. Returns the answer with source citations and confidence score
    5. Stores the conversation for history tracking
    """
)
async def chat(
    request: ChatRequest,
    db: AsyncSession = Depends(get_db),
    rag_service: RAGService = Depends(get_rag_service)
) -> ChatResponse:
    """
    Answer a user's question using RAG pipeline.

    Args:
        request: Chat request with query and session_id
        db: Database session
        rag_service: RAG service instance

    Returns:
        ChatResponse with answer, sources, and confidence score

    Raises:
        HTTPException: If processing fails
    """
    try:
        logger.info(
            f"Chat request received - Session: {request.session_id}, "
            f"Query: '{request.query[:100]}...'"
        )

        # Generate answer using RAG pipeline
        result = await rag_service.generate_answer(
            query=request.query,
            session_id=request.session_id,
            db=db
        )

        # Convert to response model
        response = ChatResponse(
            answer=result["answer"],
            sources=result["sources"],
            session_id=result["session_id"],
            confidence=result["confidence"]
        )

        logger.info(
            f"Chat response generated - Session: {request.session_id}, "
            f"Confidence: {response.confidence:.2f}"
        )

        return response

    except ValueError as e:
        # Validation errors
        logger.warning(f"Validation error: {e}")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail={"error": "ValidationError", "message": str(e)}
        )

    except Exception as e:
        # Internal errors
        logger.error(f"Chat endpoint error: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "error": "InternalServerError",
                "message": "Failed to process chat request. Please try again."
            }
        )
