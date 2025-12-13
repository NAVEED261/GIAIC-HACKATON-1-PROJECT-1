# FastAPI Agent Skill

**Purpose:** Expert in building FastAPI backend with async operations, dependency injection, and RESTful API design for RAG chatbot.

## Expertise

- FastAPI 0.104+ framework architecture
- Async/await patterns for I/O operations
- Pydantic v2 models for request/response validation
- Dependency injection with `Depends()`
- CORS middleware configuration
- OpenAPI/Swagger auto-documentation
- Exception handling and error responses
- Structured logging with Python `logging` module

## Project Structure Pattern

```
chatbot-backend/
├── app/
│   ├── __init__.py
│   ├── main.py                    # FastAPI app initialization
│   ├── api/
│   │   ├── __init__.py
│   │   ├── routes/
│   │   │   ├── __init__.py
│   │   │   ├── chat.py           # POST /api/v1/chat
│   │   │   ├── health.py         # GET /api/v1/health
│   │   │   └── history.py        # GET /api/v1/history/{session_id}
│   │   └── deps.py               # Dependency injection factories
│   ├── core/
│   │   ├── __init__.py
│   │   ├── config.py             # Settings (Pydantic BaseSettings)
│   │   └── logging.py            # Logging configuration
│   ├── models/
│   │   ├── __init__.py
│   │   ├── request.py            # ChatRequest, FeedbackRequest
│   │   └── response.py           # ChatResponse, ErrorResponse
│   ├── services/
│   │   ├── __init__.py
│   │   ├── rag.py                # RAG pipeline orchestration
│   │   ├── embeddings.py         # OpenAI embeddings wrapper
│   │   └── vector_search.py      # Qdrant search wrapper
│   └── db/
│       ├── __init__.py
│       ├── session.py            # Database connection
│       └── models.py             # SQLAlchemy models
├── tests/
│   ├── __init__.py
│   ├── test_api.py
│   └── test_services.py
├── pyproject.toml                # Dependencies (Poetry/pip)
├── .env.example
└── README.md
```

## Code Patterns

### 1. Main Application Setup

```python
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.core.config import settings
from app.api.routes import chat, health, history

app = FastAPI(
    title="Physical AI Chatbot API",
    description="RAG-powered chatbot for Physical AI Textbook",
    version="1.0.0",
    docs_url="/api/docs",
    redoc_url="/api/redoc"
)

# CORS for GitHub Pages
app.add_middleware(
    CORSMiddleware,
    allow_origins=[settings.FRONTEND_URL],
    allow_credentials=True,
    allow_methods=["GET", "POST"],
    allow_headers=["*"],
)

# Register routes
app.include_router(health.router, prefix="/api/v1", tags=["health"])
app.include_router(chat.router, prefix="/api/v1", tags=["chat"])
app.include_router(history.router, prefix="/api/v1", tags=["history"])
```

### 2. Pydantic Models (Request/Response)

```python
from pydantic import BaseModel, Field
from typing import List, Optional

class ChatRequest(BaseModel):
    query: str = Field(..., min_length=1, max_length=500)
    session_id: str = Field(..., min_length=1)
    selected_text: Optional[str] = Field(None, max_length=2000)

class ChatResponse(BaseModel):
    answer: str
    sources: List[dict]  # [{"chapter": "...", "score": 0.85}]
    session_id: str
    confidence: float = Field(..., ge=0.0, le=1.0)
```

### 3. Async Route Handler

```python
from fastapi import APIRouter, Depends, HTTPException
from app.models.request import ChatRequest
from app.models.response import ChatResponse
from app.services.rag import RAGService
from app.api.deps import get_rag_service

router = APIRouter()

@router.post("/chat", response_model=ChatResponse)
async def chat_endpoint(
    request: ChatRequest,
    rag_service: RAGService = Depends(get_rag_service)
):
    try:
        response = await rag_service.generate_answer(
            query=request.query,
            session_id=request.session_id,
            context=request.selected_text
        )
        return response
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
```

### 4. Dependency Injection

```python
# app/api/deps.py
from app.services.rag import RAGService
from app.services.embeddings import EmbeddingService
from app.services.vector_search import VectorSearchService
from app.core.config import settings

def get_embedding_service() -> EmbeddingService:
    return EmbeddingService(api_key=settings.OPENAI_API_KEY)

def get_vector_search_service() -> VectorSearchService:
    return VectorSearchService(
        url=settings.QDRANT_URL,
        api_key=settings.QDRANT_API_KEY
    )

def get_rag_service(
    embedding_service: EmbeddingService = Depends(get_embedding_service),
    vector_service: VectorSearchService = Depends(get_vector_search_service)
) -> RAGService:
    return RAGService(embedding_service, vector_service)
```

### 5. Configuration Management

```python
from pydantic_settings import BaseSettings

class Settings(BaseSettings):
    OPENAI_API_KEY: str
    QDRANT_URL: str
    QDRANT_API_KEY: str
    DATABASE_URL: str
    FRONTEND_URL: str = "https://naveed261.github.io"
    LOG_LEVEL: str = "INFO"

    class Config:
        env_file = ".env"
        case_sensitive = True

settings = Settings()
```

### 6. Error Handling

```python
from fastapi import Request
from fastapi.responses import JSONResponse

@app.exception_handler(HTTPException)
async def http_exception_handler(request: Request, exc: HTTPException):
    return JSONResponse(
        status_code=exc.status_code,
        content={
            "error": exc.detail,
            "status_code": exc.status_code,
            "path": str(request.url)
        }
    )
```

## Best Practices

1. **Always use async/await** for database and external API calls
2. **Type hints everywhere** - enforced with `mypy`
3. **Pydantic models** for all request/response validation
4. **Dependency injection** for services (testability)
5. **Structured logging** with request IDs
6. **Rate limiting** using `slowapi` middleware
7. **Health check endpoint** for monitoring
8. **OpenAPI docs** auto-generated at `/api/docs`

## Testing

```python
from fastapi.testclient import TestClient
from app.main import app

client = TestClient(app)

def test_health_endpoint():
    response = client.get("/api/v1/health")
    assert response.status_code == 200
    assert response.json()["status"] == "healthy"

def test_chat_endpoint():
    response = client.post("/api/v1/chat", json={
        "query": "What is ROS 2?",
        "session_id": "test-123"
    })
    assert response.status_code == 200
    assert "answer" in response.json()
```

## Deployment Commands

```bash
# Install dependencies
pip install fastapi[all] uvicorn[standard]

# Run development server
uvicorn app.main:app --reload --port 8000

# Run production server
uvicorn app.main:app --host 0.0.0.0 --port 8000 --workers 4

# Run with Docker
docker build -t chatbot-backend .
docker run -p 8000:8000 chatbot-backend
```

## When to Use This Agent

- Building REST API endpoints for chatbot
- Setting up FastAPI project structure
- Implementing request/response validation
- Configuring CORS and middleware
- Creating health check and monitoring endpoints
- Writing API tests with TestClient
