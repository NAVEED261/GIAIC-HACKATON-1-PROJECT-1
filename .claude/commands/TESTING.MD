● LOCAL TESTING - 2 TERMINALS:

  ---
  🚀 SETUP:

  TERMINAL 1 - BACKEND

  cd chatbot-backend
  python -m venv .venv
  .venv\Scripts\activate/uv venv
  pip install -r requirements.txt
  uvicorn app.main:app --reload   or direct ...   Dobara start kro: uvicorn app.main:app --host 127.0.0.1 --port 8000
  3. Test kro: curl http://127.0.0.1:8000/api/v1/health
  4. Browser: http://127.0.0.1:8000/api/docs
  5. Frontend: npm start (separate terminal)
  ✅ Running on: http://127.0.0.1:8000

  ---
  TERMINAL 2 - FRONTEND

  cd physical-ai-textbook
  npm install
  npm start
  ✅ Running on: http://localhost:3000
  or 
  Backend: http://127.0.0.1:8000/api/v1/health
  ✅ Frontend: http://localhost:3000/GIAIC-HACKATON-1-PROJECT-1/

  ---
  📝 CHANGE .env (IMPORTANT):

  # Update this line in chatbot-backend/.env
  FRONTEND_URL=http://localhost:3000

  ---
  ✅ TESTING:

  1. Browser mein http://localhost:3000 kolo
  2. Chatbot widget click karo
  3. Question pocho
  4. Backend local server se answer aayega

  ---
  📊 PARALLEL RUN:

  Terminal 1: Backend (8000)  ← FastAPI running
  Terminal 2: Frontend (3000) ← Docusaurus running
                      ↓
              Browser: localhost:3000
                      ↓
              Chatbot → calls http://127.0.0.1:8000

  ---
  Bas! 2 terminals + 2 commands = Full stack local testing! ✅
