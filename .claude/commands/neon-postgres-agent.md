# Neon Postgres Agent Skill

**Purpose:** Expert in Neon Serverless Postgres for chat history storage with connection pooling and async operations.

## Expertise

- Neon Serverless Postgres (free tier)
- SQLAlchemy 2.0 with async support
- Connection pooling with `asyncpg`
- Database migrations with Alembic
- Session management and chat history
- PostgreSQL JSON columns for metadata
- Index optimization for queries
- SSL connections and security

## Neon Setup

### 1. Create Free Database

```bash
# Sign up at https://neon.tech
# Create project: "physical-ai-chatbot"
# Create database: "chatbot_db"
# Get connection string:
# postgresql://user:password@ep-xxx.us-east-2.aws.neon.tech/chatbot_db?sslmode=require
```

### 2. Environment Variables

```bash
DATABASE_URL="postgresql+asyncpg://user:password@ep-xxx.us-east-2.aws.neon.tech/chatbot_db?sslmode=require"
```

## Database Schema

### Tables

```sql
-- Chat sessions
CREATE TABLE chat_sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id VARCHAR(255) UNIQUE NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    last_activity TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    metadata JSONB DEFAULT '{}'::jsonb
);

-- Chat messages
CREATE TABLE chat_messages (
    id BIGSERIAL PRIMARY KEY,
    session_id VARCHAR(255) NOT NULL REFERENCES chat_sessions(session_id) ON DELETE CASCADE,
    role VARCHAR(50) NOT NULL,  -- 'user' or 'assistant'
    content TEXT NOT NULL,
    sources JSONB DEFAULT '[]'::jsonb,  -- Retrieved chunks metadata
    confidence FLOAT,
    tokens_used INTEGER,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    CONSTRAINT valid_role CHECK (role IN ('user', 'assistant'))
);

-- Indexes for performance
CREATE INDEX idx_chat_messages_session_id ON chat_messages(session_id);
CREATE INDEX idx_chat_messages_created_at ON chat_messages(created_at DESC);
CREATE INDEX idx_chat_sessions_last_activity ON chat_sessions(last_activity DESC);
```

## Code Patterns

### 1. SQLAlchemy Models

```python
from sqlalchemy import Column, String, Text, Float, Integer, DateTime, JSON, BigInteger
from sqlalchemy.dialects.postgresql import UUID, JSONB
from sqlalchemy.ext.asyncio import AsyncAttrs
from sqlalchemy.orm import DeclarativeBase, relationship
from datetime import datetime
import uuid

class Base(AsyncAttrs, DeclarativeBase):
    pass

class ChatSession(Base):
    __tablename__ = "chat_sessions"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_id = Column(String(255), unique=True, nullable=False, index=True)
    created_at = Column(DateTime(timezone=True), default=datetime.utcnow)
    last_activity = Column(DateTime(timezone=True), default=datetime.utcnow, onupdate=datetime.utcnow)
    metadata = Column(JSONB, default={})

    # Relationship
    messages = relationship("ChatMessage", back_populates="session", cascade="all, delete-orphan")

class ChatMessage(Base):
    __tablename__ = "chat_messages"

    id = Column(BigInteger, primary_key=True, autoincrement=True)
    session_id = Column(String(255), nullable=False, index=True)
    role = Column(String(50), nullable=False)  # 'user' or 'assistant'
    content = Column(Text, nullable=False)
    sources = Column(JSONB, default=[])
    confidence = Column(Float)
    tokens_used = Column(Integer)
    created_at = Column(DateTime(timezone=True), default=datetime.utcnow)

    # Relationship
    session = relationship("ChatSession", back_populates="messages")
```

### 2. Database Connection

```python
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession, async_sessionmaker
from app.core.config import settings

# Create async engine
engine = create_async_engine(
    settings.DATABASE_URL,
    echo=False,  # Set to True for SQL query logging
    pool_size=10,
    max_overflow=20,
    pool_pre_ping=True,  # Verify connections before use
    pool_recycle=3600,  # Recycle connections after 1 hour
)

# Session factory
AsyncSessionLocal = async_sessionmaker(
    engine,
    class_=AsyncSession,
    expire_on_commit=False
)

# Dependency for FastAPI
async def get_db() -> AsyncSession:
    async with AsyncSessionLocal() as session:
        yield session
```

### 3. Database Operations Service

```python
from sqlalchemy import select, update
from sqlalchemy.ext.asyncio import AsyncSession
from app.db.models import ChatSession, ChatMessage
from typing import List, Optional, Dict
from datetime import datetime

class ChatHistoryService:
    def __init__(self, db: AsyncSession):
        self.db = db

    async def create_session(self, session_id: str, metadata: Dict = None) -> ChatSession:
        """Create new chat session."""
        session = ChatSession(
            session_id=session_id,
            metadata=metadata or {}
        )
        self.db.add(session)
        await self.db.commit()
        await self.db.refresh(session)
        return session

    async def get_or_create_session(self, session_id: str) -> ChatSession:
        """Get existing session or create new one."""
        stmt = select(ChatSession).where(ChatSession.session_id == session_id)
        result = await self.db.execute(stmt)
        session = result.scalar_one_or_none()

        if not session:
            session = await self.create_session(session_id)

        return session

    async def add_message(
        self,
        session_id: str,
        role: str,
        content: str,
        sources: List[Dict] = None,
        confidence: float = None,
        tokens_used: int = None
    ) -> ChatMessage:
        """Add message to chat history."""
        # Ensure session exists
        await self.get_or_create_session(session_id)

        message = ChatMessage(
            session_id=session_id,
            role=role,
            content=content,
            sources=sources or [],
            confidence=confidence,
            tokens_used=tokens_used
        )
        self.db.add(message)

        # Update session last_activity
        stmt = (
            update(ChatSession)
            .where(ChatSession.session_id == session_id)
            .values(last_activity=datetime.utcnow())
        )
        await self.db.execute(stmt)

        await self.db.commit()
        await self.db.refresh(message)
        return message

    async def get_session_history(
        self,
        session_id: str,
        limit: int = 50
    ) -> List[ChatMessage]:
        """Get chat history for session."""
        stmt = (
            select(ChatMessage)
            .where(ChatMessage.session_id == session_id)
            .order_by(ChatMessage.created_at.asc())
            .limit(limit)
        )
        result = await self.db.execute(stmt)
        return result.scalars().all()

    async def get_recent_sessions(self, limit: int = 10) -> List[ChatSession]:
        """Get recent active sessions."""
        stmt = (
            select(ChatSession)
            .order_by(ChatSession.last_activity.desc())
            .limit(limit)
        )
        result = await self.db.execute(stmt)
        return result.scalars().all()
```

### 4. FastAPI Integration

```python
from fastapi import APIRouter, Depends
from sqlalchemy.ext.asyncio import AsyncSession
from app.db.session import get_db
from app.db.operations import ChatHistoryService
from app.models.response import HistoryResponse

router = APIRouter()

@router.get("/history/{session_id}", response_model=HistoryResponse)
async def get_chat_history(
    session_id: str,
    limit: int = 50,
    db: AsyncSession = Depends(get_db)
):
    """Get chat history for a session."""
    service = ChatHistoryService(db)
    messages = await service.get_session_history(session_id, limit)

    return {
        "session_id": session_id,
        "messages": [
            {
                "role": msg.role,
                "content": msg.content,
                "sources": msg.sources,
                "confidence": msg.confidence,
                "timestamp": msg.created_at.isoformat()
            }
            for msg in messages
        ]
    }
```

### 5. Alembic Migrations

```bash
# Install Alembic
pip install alembic

# Initialize Alembic
alembic init alembic

# Configure alembic.ini
# sqlalchemy.url = postgresql+asyncpg://...

# Create migration
alembic revision --autogenerate -m "Create chat tables"

# Run migrations
alembic upgrade head

# Rollback
alembic downgrade -1
```

**Migration Template:**

```python
# alembic/versions/xxx_create_chat_tables.py
from alembic import op
import sqlalchemy as sa
from sqlalchemy.dialects.postgresql import UUID, JSONB

def upgrade():
    op.create_table(
        'chat_sessions',
        sa.Column('id', UUID(as_uuid=True), primary_key=True),
        sa.Column('session_id', sa.String(255), unique=True, nullable=False),
        sa.Column('created_at', sa.DateTime(timezone=True), server_default=sa.text('NOW()')),
        sa.Column('last_activity', sa.DateTime(timezone=True), server_default=sa.text('NOW()')),
        sa.Column('metadata', JSONB, default={})
    )

    op.create_table(
        'chat_messages',
        sa.Column('id', sa.BigInteger, primary_key=True),
        sa.Column('session_id', sa.String(255), nullable=False),
        sa.Column('role', sa.String(50), nullable=False),
        sa.Column('content', sa.Text, nullable=False),
        sa.Column('sources', JSONB, default=[]),
        sa.Column('confidence', sa.Float),
        sa.Column('tokens_used', sa.Integer),
        sa.Column('created_at', sa.DateTime(timezone=True), server_default=sa.text('NOW()'))
    )

    op.create_index('idx_chat_messages_session_id', 'chat_messages', ['session_id'])

def downgrade():
    op.drop_table('chat_messages')
    op.drop_table('chat_sessions')
```

## RAG Integration

```python
# In RAG service, save messages to database
async def generate_answer(
    self,
    query: str,
    session_id: str,
    db: AsyncSession,
    context: Optional[str] = None
) -> Dict:
    """Full RAG pipeline with database logging."""
    # Save user message
    history_service = ChatHistoryService(db)
    await history_service.add_message(
        session_id=session_id,
        role="user",
        content=query
    )

    # Generate answer (RAG pipeline)
    result = await self._generate_rag_answer(query)

    # Save assistant message
    await history_service.add_message(
        session_id=session_id,
        role="assistant",
        content=result["answer"],
        sources=result["sources"],
        confidence=result["confidence"],
        tokens_used=result["usage"]["total_tokens"]
    )

    return result
```

## Best Practices

1. **Use connection pooling** for performance (pool_size=10)
2. **Async operations everywhere** with SQLAlchemy 2.0
3. **Index foreign keys** (session_id) for fast queries
4. **JSONB for flexible metadata** (sources, user preferences)
5. **Migrations with Alembic** for schema versioning
6. **Cascade deletes** to clean up messages when sessions deleted
7. **SSL mode required** for Neon connections
8. **Timezone-aware timestamps** (DateTime(timezone=True))

## Testing

```python
import pytest
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession, async_sessionmaker
from app.db.models import Base, ChatSession, ChatMessage
from app.db.operations import ChatHistoryService

@pytest.fixture
async def db_session():
    engine = create_async_engine("sqlite+aiosqlite:///:memory:")
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)

    AsyncSessionLocal = async_sessionmaker(engine, class_=AsyncSession)
    async with AsyncSessionLocal() as session:
        yield session

@pytest.mark.asyncio
async def test_create_session(db_session):
    service = ChatHistoryService(db_session)
    session = await service.create_session("test-123")

    assert session.session_id == "test-123"
    assert session.id is not None

@pytest.mark.asyncio
async def test_add_message(db_session):
    service = ChatHistoryService(db_session)

    message = await service.add_message(
        session_id="test-123",
        role="user",
        content="What is ROS 2?"
    )

    assert message.role == "user"
    assert message.content == "What is ROS 2?"
```

## When to Use This Agent

- Setting up Neon Postgres database
- Creating SQLAlchemy models and schemas
- Implementing database operations (CRUD)
- Writing Alembic migrations
- Configuring connection pooling
- Testing database operations
- Optimizing queries with indexes
