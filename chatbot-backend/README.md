# RAG Chatbot Backend

RAG-powered chatbot backend for the Physical AI Textbook, built with FastAPI, OpenAI, Qdrant Cloud, and Neon Postgres.

## Features

- **FastAPI** async REST API with automatic OpenAPI documentation
- **RAG Pipeline** combining vector search (Qdrant) + LLM (OpenAI)
- **Chat History** persisted in Neon Postgres
- **Rate Limiting** (100 requests/hour per session)
- **Test Coverage** ≥80% with pytest

## Tech Stack

- **Framework:** FastAPI 0.104+
- **Database:** Neon Postgres (serverless)
- **Vector DB:** Qdrant Cloud (free tier 1GB)
- **LLM:** OpenAI (gpt-4o-mini for chat, text-embedding-3-small for embeddings)
- **Testing:** Pytest with async support
- **Deployment:** Railway (free tier)

## Setup Instructions

### 1. Install Dependencies

```bash
# Install Poetry (if not already installed)
pip install poetry

# Install dependencies
cd chatbot-backend
poetry install

# Activate virtual environment
poetry shell
```

### 2. Configure Environment Variables

```bash
# Copy example environment file
cp .env.example .env

# Edit .env with your credentials:
# - OPENAI_API_KEY (from https://platform.openai.com/api-keys)
# - QDRANT_URL and QDRANT_API_KEY (from https://cloud.qdrant.io)
# - DATABASE_URL (from https://neon.tech)
```

### 3. Run Database Migrations

```bash
# Run Alembic migrations
alembic upgrade head
```

### 4. Ingest Textbook Documents

```bash
# Run document ingestion script
python scripts/ingest_documents.py
```

### 5. Start Development Server

```bash
# Run with uvicorn
uvicorn app.main:app --reload --port 8000

# Or run with poetry
poetry run uvicorn app.main:app --reload --port 8000
```

Visit http://localhost:8000/api/docs for interactive API documentation.

## API Endpoints

### POST /api/v1/chat

Ask a question and get an AI-generated answer with source citations.

**Request:**
```json
{
  "query": "What is ROS 2?",
  "session_id": "session-1234567890-abc"
}
```

**Response:**
```json
{
  "answer": "ROS 2 is an open-source framework...",
  "sources": [
    {"chapter": "ROS 2 Fundamentals", "module": 1, "week": 3, "score": 0.92}
  ],
  "session_id": "session-1234567890-abc",
  "confidence": 0.92
}
```

### GET /api/v1/history/{session_id}

Retrieve chat history for a session.

### GET /api/v1/health

Health check endpoint.

## Testing

```bash
# Run all tests
pytest

# Run with coverage
pytest --cov=app tests/

# Run specific test file
pytest tests/test_rag_pipeline.py
```

## Deployment

### Railway Deployment

1. Push to GitHub
2. Connect Railway to your GitHub repo
3. Configure environment variables in Railway dashboard
4. Deploy automatically on push to `main`

### Manual Deployment

```bash
# Build Docker image
docker build -t chatbot-backend .

# Run container
docker run -p 8000:8000 --env-file .env chatbot-backend
```

## Project Structure

```
chatbot-backend/
├── app/
│   ├── api/routes/      # FastAPI route handlers
│   ├── services/        # Business logic (RAG, embeddings, vector search)
│   ├── models/          # Pydantic models (request/response)
│   ├── db/              # Database models and operations
│   └── core/            # Configuration and logging
├── tests/               # Pytest tests
├── scripts/             # Utility scripts (ingestion, etc.)
├── alembic/             # Database migrations
├── pyproject.toml       # Dependencies
└── Dockerfile           # Container build
```

## Documentation

- **OpenAPI Docs:** http://localhost:8000/api/docs
- **ReDoc:** http://localhost:8000/api/redoc
- **Specification:** `../specs/2-rag-chatbot/spec.md`
- **Architecture Plan:** `../specs/2-rag-chatbot/plan.md`
- **Implementation Tasks:** `../specs/2-rag-chatbot/tasks.md`

## License

Part of the Physical AI Textbook project.
