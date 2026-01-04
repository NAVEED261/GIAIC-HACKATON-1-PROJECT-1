"""
FastAPI application for RAG Chatbot Backend.

This is the main entry point for the Physical AI Textbook chatbot API.
"""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager

from app.core.logging import setup_logging, get_logger
from app.core.config import settings
from app.db.session import engine, close_db
from app.middleware.rate_limit import RateLimitMiddleware
from app.api.routes import chat, history, auth

# Setup logging
setup_logging()
logger = get_logger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Application lifespan manager for startup and shutdown events.

    Startup:
    - Initialize database connection
    - Log application start

    Shutdown:
    - Close database connections
    - Log application shutdown
    """
    # Startup
    logger.info("Starting Physical AI Chatbot API...")
    logger.info(f"OpenAI Model: {settings.OPENAI_CHAT_MODEL}")
    logger.info(f"Qdrant Collection: {settings.QDRANT_COLLECTION_NAME}")
    logger.info(f"Rate Limit: {settings.RATE_LIMIT_REQUESTS} req/{settings.RATE_LIMIT_WINDOW}s")

    yield

    # Shutdown
    logger.info("Shutting down Physical AI Chatbot API...")
    await close_db()
    logger.info("Application shutdown complete")


# Create FastAPI app
app = FastAPI(
    title="Physical AI Chatbot API",
    description="RAG-powered chatbot for Physical AI Textbook",
    version="1.0.0",
    docs_url="/api/docs",
    redoc_url="/api/redoc",
    lifespan=lifespan
)

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.ALLOWED_ORIGINS,
    allow_credentials=True,
    allow_methods=["GET", "POST"],
    allow_headers=["*"],
)

# Rate limiting middleware
app.add_middleware(RateLimitMiddleware)

# Include API routers
app.include_router(auth.router)
app.include_router(chat.router)
app.include_router(history.router)


@app.get("/")
async def root():
    """Root endpoint."""
    return {
        "message": "Physical AI Chatbot API",
        "version": "1.0.0",
        "docs": "/api/docs",
        "endpoints": {
            "chat": "/api/v1/chat",
            "history": "/api/v1/history/{session_id}",
            "health": "/api/v1/health"
        }
    }


@app.get("/api/v1/health")
async def health_check():
    """Health check endpoint."""
    return {
        "status": "healthy",
        "service": "chatbot-backend",
        "version": "1.0.0"
    }


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
