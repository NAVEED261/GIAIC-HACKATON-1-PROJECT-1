"""
Application configuration using Pydantic settings - OPTIMIZED FOR SPEED
"""

from pydantic_settings import BaseSettings, SettingsConfigDict
from pydantic import field_validator

class Settings(BaseSettings):
    """Application settings - PERFORMANCE OPTIMIZED"""

    # API Configuration
    API_V1_PREFIX: str = "/api/v1"
    PROJECT_NAME: str = "Physical AI Chatbot API"
    VERSION: str = "1.0.0"

    # OpenAI Configuration
    OPENAI_API_KEY: str = "test-key-for-local-development"
    OPENAI_EMBEDDING_MODEL: str = "text-embedding-3-small"
    OPENAI_CHAT_MODEL: str = "gpt-4o-mini"
    OPENAI_TEMPERATURE: float = 0.3
    OPENAI_MAX_TOKENS: int = 200  # Ultra-fast: 200 tokens max (80% faster)

    # Qdrant Configuration - SPEED OPTIMIZED
    QDRANT_URL: str = "http://localhost:6333"
    QDRANT_API_KEY: str = "test-key-for-local-development"
    QDRANT_COLLECTION_NAME: str = "textbook_chunks"
    QDRANT_VECTOR_SIZE: int = 1536
    QDRANT_SEARCH_LIMIT: int = 1  # Ultra-fast: Top 1 result only
    QDRANT_SCORE_THRESHOLD: float = 0.15  # Ultra-fast: Very low threshold

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

    # Rate Limiting
    RATE_LIMIT_REQUESTS: int = 100
    RATE_LIMIT_WINDOW: int = 3600

    # RAG Configuration - SPEED OPTIMIZED
    CHUNK_SIZE: int = 800
    CHUNK_OVERLAP: int = 100
    MAX_CONTEXT_CHUNKS: int = 3  # Speed: 3 chunks only
    MIN_CONFIDENCE_THRESHOLD: float = 0.2

    # Caching Configuration - NEW
    ENABLE_RESPONSE_CACHE: bool = True
    CACHE_TTL: int = 3600  # 1 hour

    # Logging
    LOG_LEVEL: str = "INFO"
    LOG_FORMAT: str = "json"

    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        case_sensitive=True
    )

    @field_validator('DATABASE_URL')
    @classmethod
    def validate_database_url(cls, v: str) -> str:
        if v.startswith("postgresql://"):
            v = v.replace("postgresql://", "postgresql+asyncpg://", 1)
        if "channel_binding=require" in v:
            v = v.replace("&channel_binding=require", "").replace("?channel_binding=require&", "?").replace("?channel_binding=require", "")
        return v

settings = Settings()
