"""
Application configuration using Pydantic settings.

Environment variables are loaded from .env file or system environment.
"""

from pydantic_settings import BaseSettings, SettingsConfigDict
from typing import Optional


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # API Configuration
    API_V1_PREFIX: str = "/api/v1"
    PROJECT_NAME: str = "Physical AI Chatbot API"
    VERSION: str = "1.0.0"

    # OpenAI Configuration
    OPENAI_API_KEY: str = "test-key-for-local-development"
    OPENAI_EMBEDDING_MODEL: str = "text-embedding-3-small"
    OPENAI_CHAT_MODEL: str = "gpt-4o-mini"
    OPENAI_TEMPERATURE: float = 0.3
    OPENAI_MAX_TOKENS: int = 1000

    # Qdrant Configuration
    QDRANT_URL: str = "http://localhost:6333"
    QDRANT_API_KEY: str = "test-key-for-local-development"
    QDRANT_COLLECTION_NAME: str = "textbook_chunks"
    QDRANT_VECTOR_SIZE: int = 1536  # text-embedding-3-small dimension
    QDRANT_SEARCH_LIMIT: int = 5
    QDRANT_SCORE_THRESHOLD: float = 0.7

    # Database Configuration
    DATABASE_URL: str = "sqlite+aiosqlite:///./chatbot.db"
    DATABASE_POOL_SIZE: int = 10
    DATABASE_MAX_OVERFLOW: int = 20

    # CORS Configuration
    FRONTEND_URL: str = "https://naveed261.github.io"
    ALLOWED_ORIGINS: list[str] = [
        "https://naveed261.github.io",
        "http://localhost:3000",
        "http://localhost:8000"
    ]

    # Rate Limiting Configuration
    RATE_LIMIT_REQUESTS: int = 100  # requests per hour
    RATE_LIMIT_WINDOW: int = 3600  # 1 hour in seconds

    # RAG Configuration
    CHUNK_SIZE: int = 800  # tokens
    CHUNK_OVERLAP: int = 100  # tokens
    MAX_CONTEXT_CHUNKS: int = 5
    MIN_CONFIDENCE_THRESHOLD: float = 0.7

    # Logging Configuration
    LOG_LEVEL: str = "INFO"
    LOG_FORMAT: str = "json"  # json or text

    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        case_sensitive=True
    )


# Create global settings instance
settings = Settings()
