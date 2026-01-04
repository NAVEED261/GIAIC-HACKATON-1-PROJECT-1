
# Local Testing Guide - Complete Stack

## Quick Start (Copy-Paste Commands)

### Terminal 1: Backend Server
```bash
cd chatbot-backend
python test_backend.py
# Output: [OK] ALL TESTS PASSED - BACKEND IS READY!

# Then start server:
uvicorn app.main:app --host 127.0.0.1 --port 8000
# Output: INFO: Uvicorn running on http://127.0.0.1:8000
```

### Terminal 2: Frontend Server
```bash
cd physical-ai-textbook
npm start
# Output: Local: http://localhost:3000
```

### Terminal 3: Test Everything
```bash
# Test backend health
curl http://127.0.0.1:8000/api/v1/health

# Test frontend (browser)
open http://localhost:3000
```

---

## What to Test

### 1. Backend API (Terminal 1)
Visit: **http://127.0.0.1:8000/api/docs**

**Test endpoints:**
- GET `/api/v1/health` → Should return 200 OK
- POST `/api/v1/chat` → Test chatbot

**Sample request body:**
```json
{
  "query": "What is ROS 2?",
  "session_id": "test-session-1"
}
```

**Expected response:**
```json
{
  "answer": "ROS 2 is...",
  "sources": [...],
  "session_id": "test-session-1",
  "confidence": 0.75
}
```

---

### 2. Frontend Docusaurus (Terminal 2)
Visit: **http://localhost:3000**

**Check:**
- ✅ Homepage loads
- ✅ Sidebar with 7 modules visible
- ✅ Chatbot button appears (bottom-right)
- ✅ Click chatbot → Widget opens
- ✅ Type query → Gets answer

---

### 3. Full Integration Test

1. **Open frontend:** http://localhost:3000
2. **Open chatbot widget:** Click button (bottom-right)
3. **Type query:** "ROS 2 Actions"
4. **Expected:**
   - ⏳ Loading animation (0.5-2 seconds)
   - 📝 Answer appears
   - 📚 Sources shown
   - ⭐ Confidence score displayed

---

## Troubleshooting

### Backend won't start
```bash
# Check if port 8000 is busy
netstat -ano | findstr :8000

# Kill process (if needed)
taskkill /PID <process_id> /F

# Then retry
uvicorn app.main:app --host 127.0.0.1 --port 8000
```

### Chatbot returns no answer
1. Check `.env` has valid OpenAI API key
2. Check `.env` has valid Qdrant credentials
3. Check backend logs for errors
4. Try health endpoint: `curl http://127.0.0.1:8000/api/v1/health`

### Frontend won't load
```bash
# Clear npm cache and reinstall
cd physical-ai-textbook
rm -r node_modules package-lock.json
npm install
npm start
```

### Port already in use
```bash
# If port 3000 busy:
npm start -- --port 3001

# If port 8000 busy:
uvicorn app.main:app --port 8001
```

---

## Key Endpoints

| Endpoint | Method | Purpose |
|----------|--------|---------|
| `/` | GET | Root/Home |
| `/api/v1/health` | GET | Health check |
| `/api/v1/chat` | POST | Send query |
| `/api/v1/history/{session_id}` | GET | Chat history |
| `/api/docs` | GET | Interactive API docs |

---

## Environment Variables (.env)

Required for chatbot to work:
```bash
OPENAI_API_KEY=sk-proj-...          # Your OpenAI key
QDRANT_URL=https://...               # Your Qdrant cloud URL
QDRANT_API_KEY=eyJ...                # Your Qdrant API key
DATABASE_URL=sqlite+aiosqlite:///./chatbot.db  # Local SQLite
```

---

## Performance Expectations

| Component | Time | Status |
|-----------|------|--------|
| Backend startup | 2-3 sec | ✅ Normal |
| Frontend startup | 5-10 sec | ✅ Normal |
| Query response | 1-2 sec | ✅ Optimized |
| Chatbot widget load | <500ms | ✅ Fast |

---

## Files Modified This Session

1. **chatbot-backend/app/db/session.py** - SQLAlchemy fix
2. **chatbot-backend/.env** - SQLite for local testing
3. **chatbot-backend/test_backend.py** - New verification script

---

## Next Steps After Testing

If everything works locally:
1. ✅ Push to GitHub
2. ✅ Deploy frontend to GitHub Pages
3. ✅ Deploy backend to Render.com
4. ✅ Test on live URLs

---

**Happy Testing!** 🚀
