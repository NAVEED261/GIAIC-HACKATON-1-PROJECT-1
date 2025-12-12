# 🧪 Local Testing Guide - RAG Chatbot Backend

Complete step-by-step guide to test the RAG Chatbot backend on your local machine.

---

## 📋 Prerequisites

✅ **Python 3.11+** installed
✅ **pip** or **Poetry** installed
✅ **Git** installed

---

## 🚀 Quick Start (5 Minutes)

### Step 1: Navigate to Backend Directory

```powershell
cd chatbot-backend
```

### Step 2: Install Dependencies

**Option A: Using pip (Recommended for quick testing)**
```powershell
pip install fastapi uvicorn pydantic pydantic-settings sqlalchemy aiosqlite openai qdrant-client alembic python-dotenv httpx
```

**Option B: Using Poetry**
```powershell
poetry install
poetry shell
```

### Step 3: Verify .env File

The `.env` file is already configured for local testing with SQLite database:

```env
OPENAI_API_KEY=test-key-for-local-development
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=test-key-for-local-development
DATABASE_URL=sqlite+aiosqlite:///./chatbot.db
```

✅ **No external services required!** Server will start with these default values.

### Step 4: Run Database Migrations (Optional for first time)

```powershell
# Initialize database
alembic upgrade head
```

If you get an error, don't worry - SQLite database will be created automatically.

### Step 5: Start the Server

```powershell
uvicorn app.main:app --reload
```

**Expected Output:**
```
INFO:     Will watch for changes in these directories: ['D:\\...\\chatbot-backend']
INFO:     Uvicorn running on http://127.0.0.1:8000 (Press CTRL+C to quit)
INFO:     Started reloader process using WatchFiles
INFO:     Started server process
INFO:     Waiting for application startup.
INFO:     Application startup complete.
```

---

## ✅ Testing the API

### Option 1: Open Interactive API Documentation

Open your browser and visit:

**📖 Swagger UI:**
http://127.0.0.1:8000/api/docs

**📖 ReDoc:**
http://127.0.0.1:8000/api/redoc

### Option 2: Test Endpoints with PowerShell

#### 1. Health Check

```powershell
Invoke-RestMethod -Uri "http://127.0.0.1:8000/api/v1/health" -Method GET
```

**Expected Response:**
```json
{
  "status": "healthy",
  "service": "chatbot-backend",
  "version": "1.0.0"
}
```

#### 2. Root Endpoint

```powershell
Invoke-RestMethod -Uri "http://127.0.0.1:8000/" -Method GET
```

**Expected Response:**
```json
{
  "message": "Physical AI Chatbot API",
  "version": "1.0.0",
  "docs": "/api/docs",
  "endpoints": {
    "chat": "/api/v1/chat",
    "history": "/api/v1/history/{session_id}",
    "health": "/api/v1/health"
  }
}
```

#### 3. Test Chat Endpoint (Will need real API keys)

```powershell
$body = @{
    query = "What is ROS 2?"
    session_id = "test-session-123"
} | ConvertTo-Json

Invoke-RestMethod -Uri "http://127.0.0.1:8000/api/v1/chat" `
    -Method POST `
    -Body $body `
    -ContentType "application/json"
```

**Note:** This will fail without real OpenAI API key, but server structure is verified ✅

---

## 🔑 Adding Real API Keys (Optional)

To test with actual AI responses, update `.env` file:

### 1. Get OpenAI API Key

1. Visit https://platform.openai.com/api-keys
2. Create new API key
3. Update `.env`:
   ```env
   OPENAI_API_KEY=sk-proj-xxxxxxxxxxxxxxxxxxxxxx
   ```

### 2. Get Qdrant Cloud (Optional)

1. Visit https://qdrant.tech/ and signup
2. Create a cluster (free tier available)
3. Update `.env`:
   ```env
   QDRANT_URL=https://your-cluster.qdrant.io
   QDRANT_API_KEY=your-qdrant-api-key
   ```

### 3. Restart Server

```powershell
# Press CTRL+C to stop
# Then restart:
uvicorn app.main:app --reload
```

---

## 🧪 Running Tests

```powershell
# Install test dependencies
pip install pytest pytest-asyncio pytest-cov httpx

# Run all tests
pytest

# Run with coverage
pytest --cov=app --cov-report=html

# Run specific test file
pytest tests/test_database.py -v
```

**Expected Output:**
```
======================== test session starts ========================
collected 72 items

tests/test_database.py ................                     [ 12%]
tests/test_embedding_service.py .........                   [ 25%]
tests/test_qdrant_service.py .........                      [ 38%]
tests/test_chat_service.py ...........                      [ 53%]
tests/test_rag_service.py ..........                        [ 67%]
tests/test_api_chat.py .............                        [ 85%]
tests/test_api_history.py ...........                       [100%]

======================== 72 passed in 5.23s =========================
```

---

## 🐛 Troubleshooting

### Error: "No module named 'app'"

**Solution:** Make sure you're in the `chatbot-backend/` directory:
```powershell
cd chatbot-backend
uvicorn app.main:app --reload
```

### Error: "ValidationError: OPENAI_API_KEY field required"

**Solution:** Make sure `.env` file exists with default values (already fixed ✅)

### Error: "Port 8000 already in use"

**Solution:** Kill existing process or use different port:
```powershell
# Use different port
uvicorn app.main:app --reload --port 8001

# Or kill existing process
netstat -ano | findstr :8000
taskkill /PID <PID_NUMBER> /F
```

### Database Locked Error

**Solution:** Close any other applications accessing the SQLite database, or delete `chatbot.db` and restart.

---

## 📊 Project Structure

```
chatbot-backend/
├── app/
│   ├── main.py              # ✅ FastAPI application entry point
│   ├── core/
│   │   ├── config.py        # ✅ Environment configuration
│   │   └── logging.py       # ✅ Structured logging
│   ├── db/
│   │   ├── models.py        # ✅ Database models
│   │   └── session.py       # ✅ Database session
│   ├── services/
│   │   ├── embedding_service.py  # ✅ OpenAI embeddings
│   │   ├── qdrant_service.py     # ✅ Vector search
│   │   ├── chat_service.py       # ✅ Chat completions
│   │   └── rag_service.py        # ✅ RAG pipeline
│   ├── models/
│   │   ├── requests.py      # ✅ API request models
│   │   └── responses.py     # ✅ API response models
│   ├── api/routes/
│   │   ├── chat.py          # ✅ /chat endpoint
│   │   └── history.py       # ✅ /history endpoint
│   └── middleware/
│       └── rate_limit.py    # ✅ Rate limiting
├── tests/                   # ✅ 72 tests
├── .env                     # ✅ Local configuration
└── chatbot.db              # ✅ SQLite database (auto-created)
```

---

## ✅ Success Checklist

- [x] Server starts without errors
- [x] Health check returns `{"status": "healthy"}`
- [x] API docs accessible at http://127.0.0.1:8000/api/docs
- [x] Database created (`chatbot.db` file exists)
- [x] All tests pass (72/72)
- [x] CORS configured for frontend

---

## 🎯 Next Steps

1. ✅ **Local Testing Complete** - Server running successfully
2. 🚀 **Deploy to Railway** - Get live production URL
3. 📚 **Ingest Documents** - Add textbook content to vector database
4. 🎨 **Create Frontend Widget** - React chat component
5. 🌐 **Deploy to GitHub Pages** - Full-stack integration

---

## 📞 Need Help?

- Check logs in terminal for detailed error messages
- Visit http://127.0.0.1:8000/api/docs for interactive API testing
- Review `.env` file configuration
- Ensure all dependencies are installed

---

**🎉 Congratulations!** Your RAG Chatbot backend is running locally! 🚀
