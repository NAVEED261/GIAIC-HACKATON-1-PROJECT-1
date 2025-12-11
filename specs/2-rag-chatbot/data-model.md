# Data Model: RAG Chatbot Integration

**Feature:** RAG Chatbot Integration
**Created:** 2025-12-11
**Version:** 1.0.0

## Overview

This document defines the data model for the RAG Chatbot system, including database entities (Neon Postgres), vector store entities (Qdrant), and API request/response models (Pydantic).

---

## Database Entities (Neon Postgres)

### Entity 1: ChatSession

Represents a conversation thread with a user.

**Purpose:** Track individual chat sessions for history retrieval and analytics.

**Table Name:** `chat_sessions`

**Attributes:**

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PRIMARY KEY, DEFAULT gen_random_uuid() | Internal database ID |
| session_id | VARCHAR(255) | UNIQUE, NOT NULL, INDEX | Client-generated session ID (stored in localStorage) |
| created_at | TIMESTAMP WITH TIME ZONE | DEFAULT NOW() | Session creation timestamp |
| last_activity | TIMESTAMP WITH TIME ZONE | DEFAULT NOW(), ON UPDATE NOW() | Last message timestamp |
| metadata | JSONB | DEFAULT '{}' | User agent, IP, client info (analytics) |

**Relationships:**
- `HAS MANY ChatMessage` (1-to-many via session_id)

**Indexes:**
- Primary: `id` (UUID)
- Unique: `session_id` (client lookup)
- Performance: `last_activity DESC` (recent sessions query)

**SQLAlchemy Model:**
```python
from sqlalchemy import Column, String, DateTime, JSON
from sqlalchemy.dialects.postgresql import UUID, JSONB
from sqlalchemy.orm import relationship
from datetime import datetime
import uuid

class ChatSession(Base):
    __tablename__ = "chat_sessions"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_id = Column(String(255), unique=True, nullable=False, index=True)
    created_at = Column(DateTime(timezone=True), default=datetime.utcnow)
    last_activity = Column(DateTime(timezone=True), default=datetime.utcnow, onupdate=datetime.utcnow)
    metadata = Column(JSONB, default={})

    # Relationship
    messages = relationship("ChatMessage", back_populates="session", cascade="all, delete-orphan")
```

---

### Entity 2: ChatMessage

Represents a single message (user question or assistant answer) in a conversation.

**Purpose:** Store chat history for retrieval and analytics.

**Table Name:** `chat_messages`

**Attributes:**

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | BIGSERIAL | PRIMARY KEY | Auto-incrementing message ID |
| session_id | VARCHAR(255) | NOT NULL, FOREIGN KEY (chat_sessions.session_id), INDEX | Links to chat session |
| role | VARCHAR(50) | NOT NULL, CHECK ('user' OR 'assistant') | Message sender |
| content | TEXT | NOT NULL | Question or answer text |
| sources | JSONB | DEFAULT '[]' | Retrieved chunks (chapter, score) for assistant messages |
| confidence | FLOAT | NULL | Average similarity score (0.0-1.0) for assistant messages |
| tokens_used | INTEGER | NULL | Total OpenAI tokens (prompt + completion) |
| created_at | TIMESTAMP WITH TIME ZONE | DEFAULT NOW() | Message creation timestamp |

**Relationships:**
- `BELONGS TO ChatSession` (many-to-1 via session_id)

**Indexes:**
- Primary: `id` (BIGSERIAL)
- Foreign key: `session_id` (cascade delete when session deleted)
- Performance: `created_at DESC` (chronological message retrieval)

**SQLAlchemy Model:**
```python
from sqlalchemy import Column, String, Text, Float, Integer, BigInteger, DateTime
from sqlalchemy.dialects.postgresql import JSONB

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

**Example sources JSONB:**
```json
[
  {"chapter": "ROS 2 Fundamentals", "module": 1, "week": 3, "score": 0.92},
  {"chapter": "ROS 2 Architecture", "module": 1, "week": 4, "score": 0.85}
]
```

---

## Vector Store Entities (Qdrant)

### Entity 3: TextbookChunk

Represents an embedded segment of textbook content stored in Qdrant vector database.

**Purpose:** Enable semantic search for RAG context retrieval.

**Collection Name:** `textbook_chunks`

**Attributes:**

| Field | Type | Description |
|-------|------|-------------|
| id | UUID | Qdrant point ID (unique per chunk) |
| vector | float[] (1536) | OpenAI text-embedding-3-small embedding |
| payload.text | string | Chunk content (500-800 tokens) |
| payload.chapter | string | Chapter title from frontmatter |
| payload.module | integer (1-4) | Module number |
| payload.week | integer (1-13) | Week number |
| payload.file_path | string | Relative path to source markdown file |

**Qdrant Configuration:**
```python
from qdrant_client.models import Distance, VectorParams

VectorParams(
    size=1536,  # text-embedding-3-small dimension
    distance=Distance.COSINE  # cosine similarity
)
```

**Example Payload:**
```json
{
  "text": "ROS 2 (Robot Operating System 2) is an open-source framework for robot software development...",
  "chapter": "ROS 2 Fundamentals",
  "module": 1,
  "week": 3,
  "file_path": "docs/module-1-ros2/week-3-fundamentals.md"
}
```

**Search Query Example:**
```python
from qdrant_client.models import Filter, FieldCondition, MatchValue

# Search with module filter
results = client.search(
    collection_name="textbook_chunks",
    query_vector=query_embedding,  # 1536-dim float array
    limit=5,
    score_threshold=0.7,
    query_filter=Filter(
        must=[
            FieldCondition(key="module", match=MatchValue(value=1))
        ]
    )
)
```

---

## API Request/Response Models (Pydantic)

### Model 1: ChatRequest

Request payload for asking questions.

**Endpoint:** `POST /api/v1/chat`

**Pydantic Model:**
```python
from pydantic import BaseModel, Field
from typing import Optional

class ChatRequest(BaseModel):
    query: str = Field(..., min_length=1, max_length=500, description="User's question")
    session_id: str = Field(..., min_length=1, max_length=255, description="Client-generated session ID")
    selected_text: Optional[str] = Field(None, max_length=2000, description="Selected text from textbook (optional)")

    class Config:
        json_schema_extra = {
            "example": {
                "query": "What is ROS 2?",
                "session_id": "session-1234567890-abc",
                "selected_text": None
            }
        }
```

**Validation Rules:**
- `query`: Required, 1-500 characters (prevent abuse)
- `session_id`: Required, generated by frontend (localStorage)
- `selected_text`: Optional, max 2000 characters (context enhancement)

---

### Model 2: ChatResponse

Response payload for answers.

**Endpoint:** `POST /api/v1/chat` (response)

**Pydantic Model:**
```python
from pydantic import BaseModel, Field
from typing import List

class Source(BaseModel):
    chapter: str
    module: int = Field(..., ge=1, le=4)
    week: int = Field(..., ge=1, le=13)
    score: float = Field(..., ge=0.0, le=1.0)

class ChatResponse(BaseModel):
    answer: str = Field(..., description="LLM-generated answer")
    sources: List[Source] = Field(..., description="Retrieved chunks with metadata")
    session_id: str
    confidence: float = Field(..., ge=0.0, le=1.0, description="Average similarity score")

    class Config:
        json_schema_extra = {
            "example": {
                "answer": "ROS 2 is an open-source framework for robot software development...",
                "sources": [
                    {"chapter": "ROS 2 Fundamentals", "module": 1, "week": 3, "score": 0.92}
                ],
                "session_id": "session-1234567890-abc",
                "confidence": 0.92
            }
        }
```

**Validation Rules:**
- `answer`: Required, LLM-generated text
- `sources`: Required array (empty if confidence <0.7)
- `confidence`: Float 0.0-1.0 (average of source scores)

---

### Model 3: HistoryResponse

Response payload for chat history retrieval.

**Endpoint:** `GET /api/v1/history/{session_id}`

**Pydantic Model:**
```python
from pydantic import BaseModel
from typing import List, Optional
from datetime import datetime

class MessageHistory(BaseModel):
    role: str  # 'user' or 'assistant'
    content: str
    sources: Optional[List[Source]] = None  # Only for assistant messages
    confidence: Optional[float] = None  # Only for assistant messages
    timestamp: datetime

class HistoryResponse(BaseModel):
    session_id: str
    messages: List[MessageHistory]

    class Config:
        json_schema_extra = {
            "example": {
                "session_id": "session-1234567890-abc",
                "messages": [
                    {
                        "role": "user",
                        "content": "What is ROS 2?",
                        "timestamp": "2025-12-11T10:30:00Z"
                    },
                    {
                        "role": "assistant",
                        "content": "ROS 2 is...",
                        "sources": [{"chapter": "ROS 2 Fundamentals", "module": 1, "week": 3, "score": 0.92}],
                        "confidence": 0.92,
                        "timestamp": "2025-12-11T10:30:02Z"
                    }
                ]
            }
        }
```

---

### Model 4: HealthResponse

Response payload for health check.

**Endpoint:** `GET /api/v1/health`

**Pydantic Model:**
```python
from pydantic import BaseModel
from datetime import datetime

class ServiceStatus(BaseModel):
    database: str  # 'connected' or 'disconnected'
    qdrant: str  # 'connected' or 'disconnected'
    openai: str  # 'available' or 'unavailable'

class HealthResponse(BaseModel):
    status: str  # 'healthy' or 'degraded'
    timestamp: datetime
    services: ServiceStatus
```

---

### Model 5: ErrorResponse

Error response for all endpoints.

**Pydantic Model:**
```python
from pydantic import BaseModel

class ErrorResponse(BaseModel):
    error: str  # Human-readable error message
    status_code: int  # HTTP status code
    path: str  # Request path
    timestamp: datetime

    class Config:
        json_schema_extra = {
            "example": {
                "error": "Rate limit exceeded. Please try again in 30 minutes.",
                "status_code": 429,
                "path": "/api/v1/chat",
                "timestamp": "2025-12-11T10:30:00Z"
            }
        }
```

---

## Entity Relationships

```
ChatSession (1) ----< (Many) ChatMessage
   |
   +-- id (UUID)
   +-- session_id (VARCHAR, UNIQUE)
   +-- created_at (TIMESTAMP)
   +-- last_activity (TIMESTAMP)
   +-- metadata (JSONB)

ChatMessage (Many) >---- (1) ChatSession
   |
   +-- id (BIGSERIAL)
   +-- session_id (VARCHAR, FK)
   +-- role (VARCHAR: 'user'|'assistant')
   +-- content (TEXT)
   +-- sources (JSONB)
   +-- confidence (FLOAT)
   +-- tokens_used (INTEGER)
   +-- created_at (TIMESTAMP)

TextbookChunk (Qdrant)
   |
   +-- id (UUID)
   +-- vector (float[1536])
   +-- payload:
        +-- text (string)
        +-- chapter (string)
        +-- module (integer)
        +-- week (integer)
        +-- file_path (string)
```

---

## State Transitions

### ChatSession States

1. **Created:** Session ID generated by frontend, stored in localStorage
2. **Active:** User sends first message, session record created in database
3. **Idle:** No messages for >24 hours (last_activity timestamp)
4. **Archived:** Auto-archive after 30 days of inactivity (optional cleanup job)

### ChatMessage Flow

```
User Input → ChatRequest → RAG Pipeline → ChatResponse → Save to DB
                                ↓
                        Vector Search (Qdrant)
                                ↓
                        LLM Completion (OpenAI)
                                ↓
                        ChatMessage (role='assistant')
```

---

## Validation Rules

### Database Constraints

1. **chat_sessions.session_id:** UNIQUE (prevent duplicate sessions)
2. **chat_messages.role:** CHECK (role IN ('user', 'assistant'))
3. **chat_messages.session_id:** FOREIGN KEY with CASCADE DELETE
4. **chat_messages.confidence:** CHECK (confidence >= 0.0 AND confidence <= 1.0)

### Pydantic Validation

1. **ChatRequest.query:** 1-500 characters
2. **ChatRequest.selected_text:** Max 2000 characters
3. **ChatResponse.confidence:** 0.0-1.0 float
4. **Source.module:** Integer 1-4
5. **Source.week:** Integer 1-13

---

## Sample Data

### ChatSession Record
```json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "session_id": "session-1702300800-abc123",
  "created_at": "2025-12-11T10:00:00Z",
  "last_activity": "2025-12-11T10:30:00Z",
  "metadata": {
    "user_agent": "Mozilla/5.0...",
    "ip_hash": "hash_of_ip",
    "referrer": "https://naveed261.github.io/..."
  }
}
```

### ChatMessage Record (User)
```json
{
  "id": 1,
  "session_id": "session-1702300800-abc123",
  "role": "user",
  "content": "What is ROS 2?",
  "sources": [],
  "confidence": null,
  "tokens_used": null,
  "created_at": "2025-12-11T10:30:00Z"
}
```

### ChatMessage Record (Assistant)
```json
{
  "id": 2,
  "session_id": "session-1702300800-abc123",
  "role": "assistant",
  "content": "ROS 2 is an open-source framework...",
  "sources": [
    {"chapter": "ROS 2 Fundamentals", "module": 1, "week": 3, "score": 0.92},
    {"chapter": "ROS 2 Architecture", "module": 1, "week": 4, "score": 0.85}
  ],
  "confidence": 0.885,
  "tokens_used": 1523,
  "created_at": "2025-12-11T10:30:02Z"
}
```

---

## Migration Scripts

See: `specs/2-rag-chatbot/contracts/database-schema.sql` for Alembic migration template.

---

**Status:** Complete
**Version:** 1.0.0
**Last Updated:** 2025-12-11
