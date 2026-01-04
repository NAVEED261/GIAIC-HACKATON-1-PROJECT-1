"""
Async database session management using SQLAlchemy 2.0.

Provides async engine and session factory for database operations.
"""

from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession, async_sessionmaker
from sqlalchemy.pool import NullPool, StaticPool
from typing import AsyncGenerator
from ..core.config import settings
from ..core.logging import get_logger

logger = get_logger(__name__)

# Create async engine - SQLite requires StaticPool for async
engine = create_async_engine(
    settings.DATABASE_URL,
    echo=False,
    poolclass=StaticPool,  # SQLite workaround for async
    connect_args={"check_same_thread": False},  # SQLite async fix
)

# Create async session factory
AsyncSessionLocal = async_sessionmaker(
    engine,
    class_=AsyncSession,
    expire_on_commit=False,
    autocommit=False,
    autoflush=False
)


async def get_db() -> AsyncGenerator[AsyncSession, None]:
    """
    Dependency for FastAPI routes to get database session.

    Usage:
        @router.get("/endpoint")
        async def endpoint(db: AsyncSession = Depends(get_db)):
            # Use db session here
            pass

    Yields:
        AsyncSession: Database session
    """
    async with AsyncSessionLocal() as session:
        try:
            yield session
            await session.commit()
        except Exception as e:
            await session.rollback()
            logger.error(f"Database session error: {e}")
            raise
        finally:
            await session.close()


async def init_db() -> None:
    """Initialize database tables (for testing purposes only)."""
    from .models import Base

    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)

    logger.info("Database tables created successfully")


async def close_db() -> None:
    """Close database connections on application shutdown."""
    await engine.dispose()
    logger.info("Database connections closed")
