# Implementation Tasks: RAG Chatbot Integration

**Feature:** RAG Chatbot Integration
**Version:** 1.0.0
**Status:** Ready for Implementation
**Created:** 2025-12-11
**Total Tasks:** 65

## Task Summary

| Phase | User Story | Tasks | Parallelizable | Test Tasks |
|-------|-----------|-------|----------------|------------|
| Phase 1: Setup | N/A | 10 | 5 | 0 |
| Phase 2: Foundational | N/A | 8 | 4 | 3 |
| Phase 3: US1 (P1) | Ask Questions | 12 | 6 | 4 |
| Phase 4: US2 (P1) | Text Selection | 8 | 5 | 3 |
| Phase 5: US3 (P2) | Chat History | 10 | 6 | 3 |
| Phase 6: US4 (P2) | Confidence Display | 7 | 4 | 2 |
| Phase 7: Polish | N/A | 10 | 7 | 2 |
| **TOTAL** | **4 Stories** | **65** | **37** | **17** |

---

## Implementation Strategy

### MVP Scope (Minimum Viable Product)

**User Story 1 only** (Tasks T001-T030):
- Backend API with basic chat endpoint
- RAG pipeline (embeddings + vector search + LLM)
- Database for chat history
- Frontend chat widget with basic UI
- Core functionality: Ask questions, get answers with sources

**Estimated Time:** 3-4 hours
**Deployment:** Railway (backend) + GitHub Pages (frontend)

### Incremental Delivery

- **Sprint 1** (MVP): US1 - Basic question answering (T001-T030)
- **Sprint 2**: US2 - Text selection integration (T031-T038)
- **Sprint 3**: US3 + US4 - History + confidence display (T039-T055)
- **Sprint 4**: Polish & optimization (T056-T065)

---

## User Story Dependencies

```
Foundational (Phase 2)
    ↓
US1: Ask Questions (P1) ← INDEPENDENT (MVP)
    ↓
US2: Text Selection (P1) ← Depends on US1 (frontend exists)
    ↓
US3: Chat History (P2) ← Depends on US1 (database exists)
    ↓
US4: Confidence Display (P2) ← Depends on US1 (response structure exists)
```

**Parallel Opportunities:**
- US2, US3, US4 can be implemented in parallel after US1 completes
- Within each story: Multiple tasks marked `[P]` can run concurrently

---

## Phase 1: Setup & Project Initialization

**Goal:** Create backend and frontend project structures with all dependencies installed.

**Tasks:**

- [ ] T001 Create backend project directory `chatbot-backend/` with Python virtual environment
- [ ] T002 [P] Install FastAPI dependencies in `chatbot-backend/pyproject.toml` (fastapi, uvicorn, pydantic, sqlalchemy, asyncpg, openai, qdrant-client, alembic, pytest, black, mypy)
- [ ] T003 [P] Create backend directory structure: `app/{api/,services/,models/,db/,core/}`, `tests/`, `scripts/`, `alembic/`
- [ ] T004 [P] Create `.env.example` file in `chatbot-backend/` with environment variable template (OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY, DATABASE_URL, FRONTEND_URL)
- [ ] T005 [P] Create `chatbot-backend/app/main.py` with basic FastAPI app initialization and CORS middleware
- [ ] T006 [P] Create frontend chat widget directory `physical-ai-textbook/src/components/ChatWidget/`
- [ ] T007 [P] Install frontend dependencies in `physical-ai-textbook/package.json` (no additional deps needed, React already available)
- [ ] T008 Create `chatbot-backend/README.md` with setup instructions and API documentation link
- [ ] T009 Create `chatbot-backend/Dockerfile` for containerized deployment
- [ ] T010 Create `chatbot-backend/railway.toml` for Railway deployment configuration

**Parallel Execution Example (Phase 1):**
```bash
# Run simultaneously:
- T002, T003, T004, T005, T006, T007 (independent file creation)
# Sequential:
- T001 → T002-T007 → T008-T010
```

---

## Phase 2: Foundational Infrastructure (Blocking Prerequisites)

**Goal:** Set up database schema, core services, and configuration that all user stories depend on.

**Independent Test:** Database migrations run successfully, Qdrant collection created, OpenAI API connection verified.

**Tasks:**

- [ ] T011 Create Pydantic settings model in `chatbot-backend/app/core/config.py` (OpenAI key, Qdrant URL, Database URL, CORS origins)
- [ ] T012 [P] Create structured logging configuration in `chatbot-backend/app/core/logging.py` (JSON format for production)
- [ ] T013 [P] Create database models in `chatbot-backend/app/db/models.py` (ChatSession, ChatMessage with SQLAlchemy)
- [ ] T014 Create database session manager in `chatbot-backend/app/db/session.py` (async engine, sessionmaker, connection pooling)
- [ ] T015 Create Alembic migration for chat tables in `chatbot-backend/alembic/versions/001_create_chat_tables.py`
- [ ] T016 [P] Create Qdrant client wrapper in `chatbot-backend/app/services/vector_search.py` (connect, create collection, search methods)
- [ ] T017 [P] Create OpenAI embedding service in `chatbot-backend/app/services/embeddings.py` (embed_text, embed_batch methods)
- [ ] T018 [P] Test database connection and migrations with `pytest chatbot-backend/tests/test_database.py`
- [ ] T019 [P] Test Qdrant collection creation with `pytest chatbot-backend/tests/test_qdrant.py`
- [ ] T020 [P] Test OpenAI API connection with `pytest chatbot-backend/tests/test_openai.py`

**Parallel Execution Example (Phase 2):**
```bash
# Parallel (after T011 completes):
- T012, T013, T016, T017 (different services)
# Sequential:
- T011 → T014, T015 (database depends on config)
- T018, T019, T020 (tests run after services created)
```

---

## Phase 3: User Story 1 - Ask Questions About Course Content (P1)

**Story Goal:** Students can ask natural language questions and receive accurate answers with source citations in <3 seconds.

**Independent Test:**
1. POST request to `/api/v1/chat` with question "What is ROS 2?" returns answer with sources
2. Response time <3 seconds
3. Confidence score ≥0.7
4. Sources include chapter name, module, week, similarity score

**Acceptance Criteria:**
- ✅ Basic question answering with citations
- ✅ Multi-topic questions reference multiple modules
- ✅ Contextual follow-up maintains conversation context
- ✅ Source attribution shows chapter names and scores

**Tasks:**

### Backend RAG Pipeline

- [ ] T021 [US1] Create Pydantic request model in `chatbot-backend/app/models/request.py` (ChatRequest with query, session_id, selected_text)
- [ ] T022 [P] [US1] Create Pydantic response models in `chatbot-backend/app/models/response.py` (ChatResponse, Source, HealthResponse, ErrorResponse)
- [ ] T023 [P] [US1] Create document chunking utility in `chatbot-backend/app/services/chunking.py` (chunk_document with 500-800 tokens, 100-token overlap)
- [ ] T024 [US1] Create document ingestion script in `chatbot-backend/scripts/ingest_documents.py` (read markdown files, chunk, embed, upload to Qdrant)
- [ ] T025 [US1] Run ingestion script to embed 70+ textbook files into Qdrant collection
- [ ] T026 [P] [US1] Create OpenAI chat service in `chatbot-backend/app/services/chat.py` (generate_answer with context injection, temperature=0.3)
- [ ] T027 [US1] Create RAG orchestration service in `chatbot-backend/app/services/rag.py` (embed query → search Qdrant → generate answer with LLM)
- [ ] T028 [US1] Create database operations service in `chatbot-backend/app/db/operations.py` (CRUD for sessions and messages)

### API Endpoints

- [ ] T029 [US1] Create chat endpoint in `chatbot-backend/app/api/routes/chat.py` (POST /api/v1/chat with RAG pipeline integration)
- [ ] T030 [P] [US1] Create health check endpoint in `chatbot-backend/app/api/routes/health.py` (GET /api/v1/health with service status checks)

### Tests

- [ ] T031 [P] [US1] Test RAG pipeline end-to-end with `pytest chatbot-backend/tests/test_rag_pipeline.py`
- [ ] T032 [P] [US1] Test chat endpoint with mock OpenAI/Qdrant in `pytest chatbot-backend/tests/test_api.py`
- [ ] T033 [P] [US1] Test response time <3s with `pytest chatbot-backend/tests/test_performance.py`
- [ ] T034 [P] [US1] Test confidence threshold ≥0.7 filtering with `pytest chatbot-backend/tests/test_confidence.py`

**Parallel Execution Example (US1):**
```bash
# Parallel (after T021-T022 complete):
- T023, T026 (chunking and chat service independent)
- T031, T032, T033, T034 (all tests can run simultaneously)
```

**US1 Complete When:**
- [x] Chat endpoint responds with answers and sources
- [x] Response time <3 seconds
- [x] ≥80% test coverage for RAG pipeline
- [x] Manual test: "What is ROS 2?" returns relevant answer with Module 1 citations

---

## Phase 4: User Story 2 - Query Selected Text for Context-Aware Help (P1)

**Story Goal:** Students can select text from textbook and ask context-aware questions with auto-opening chat widget.

**Independent Test:**
1. Select text >10 characters on textbook page
2. Chat widget auto-opens with selected text banner
3. Ask "What does this mean?" → response references selected text
4. Clear button removes selected context

**Acceptance Criteria:**
- ✅ Text selection triggers chatbot auto-open
- ✅ Context-aware responses use selected text
- ✅ Clear selected context button functional
- ✅ Selection preservation across multiple questions

**Tasks:**

### Frontend Text Selection

- [ ] T035 [P] [US2] Create TypeScript interfaces in `physical-ai-textbook/src/components/ChatWidget/types.ts` (Message, Source, ChatRequest, ChatResponse)
- [ ] T036 [P] [US2] Create ChatWidget main component in `physical-ai-textbook/src/components/ChatWidget/ChatWidget.tsx` (floating button, panel, state management)
- [ ] T037 [US2] Implement text selection handler in `ChatWidget.tsx` (window.getSelection, auto-open on >10 chars, selected text banner)
- [ ] T038 [P] [US2] Create ChatInput component in `physical-ai-textbook/src/components/ChatWidget/ChatInput.tsx` (textarea, send button, Enter key handler)
- [ ] T039 [P] [US2] Create ChatHistory component in `physical-ai-textbook/src/components/ChatWidget/ChatHistory.tsx` (message list, auto-scroll, loading indicator)
- [ ] T040 [P] [US2] Create ChatMessage component in `physical-ai-textbook/src/components/ChatWidget/ChatMessage.tsx` (user/assistant bubbles, sources display, confidence badge)
- [ ] T041 [US2] Integrate ChatWidget into Docusaurus by creating `physical-ai-textbook/src/theme/Root.tsx` (swizzle theme, embed widget)

### Tests

- [ ] T042 [P] [US2] Test text selection handler with React Testing Library in `physical-ai-textbook/src/components/ChatWidget/__tests__/ChatWidget.test.tsx`
- [ ] T043 [P] [US2] Test selected text banner display and clear button
- [ ] T044 [P] [US2] Test API call with selected_text parameter

**Parallel Execution Example (US2):**
```bash
# Parallel:
- T035, T036, T038, T039, T040 (independent components)
- T042, T043, T044 (independent tests)
```

**US2 Complete When:**
- [x] Text selection auto-opens chat widget
- [x] Selected text visible in banner with clear button
- [x] Questions reference selected context
- [x] Manual test: Select "ROS 2 uses DDS" → ask "What is DDS?" → answer explains DDS in context of ROS 2

---

## Phase 5: User Story 3 - Access Chat History Across Sessions (P2)

**Story Goal:** Students see previous chat conversations when they return, enabling learning continuity.

**Independent Test:**
1. Ask questions in session-1
2. Close browser, reopen
3. Chat history loads from database
4. Session ID persists in localStorage

**Acceptance Criteria:**
- ✅ Session persistence across browser sessions
- ✅ Session ID stored in localStorage
- ✅ History retrieval from backend
- ✅ New session option (clear localStorage)

**Tasks:**

### Backend History Endpoint

- [ ] T045 [P] [US3] Create history endpoint in `chatbot-backend/app/api/routes/history.py` (GET /api/v1/history/{session_id} with pagination)
- [ ] T046 [P] [US3] Implement get_session_history method in `chatbot-backend/app/db/operations.py` (retrieve last 50 messages for session)
- [ ] T047 [P] [US3] Implement session creation/update in database operations (create_session, update_last_activity)

### Frontend Session Management

- [ ] T048 [P] [US3] Implement session ID generation and localStorage persistence in `ChatWidget.tsx` (generate UUID, store in localStorage)
- [ ] T049 [US3] Implement chat history loading on widget mount in `ChatWidget.tsx` (fetch from /history endpoint, populate messages state)
- [ ] T050 [P] [US3] Implement session tracking in chat endpoint to save all messages to database
- [ ] T051 [P] [US3] Add "New Session" button to ChatWidget header (clear localStorage, generate new session ID)

### Tests

- [ ] T052 [P] [US3] Test history endpoint with `pytest chatbot-backend/tests/test_history_api.py`
- [ ] T053 [P] [US3] Test session persistence with React Testing Library
- [ ] T054 [P] [US3] Test message saving to database after each chat interaction

**Parallel Execution Example (US3):**
```bash
# Parallel:
- T045, T046, T047 (backend endpoints)
- T048, T049, T051 (frontend features)
- T052, T053, T054 (independent tests)
```

**US3 Complete When:**
- [x] Chat history persists across browser sessions
- [x] Session ID stored in localStorage
- [x] History loads asynchronously on widget mount
- [x] Manual test: Ask question, close browser, reopen → history visible

---

## Phase 6: User Story 4 - Understand Answer Confidence and Sources (P2)

**Story Goal:** Students see confidence scores and source citations to judge answer reliability.

**Independent Test:**
1. Ask question "What is ROS 2?"
2. Response displays confidence score (e.g., 85%)
3. Sources list shows chapters with similarity scores
4. Low confidence (<70%) triggers fallback message

**Acceptance Criteria:**
- ✅ Confidence display in UI
- ✅ Source citations with chapter names
- ✅ Low confidence warning
- ✅ Source navigation (click to chapter)

**Tasks:**

### Backend Confidence Logic

- [ ] T055 [P] [US4] Implement confidence calculation in `chatbot-backend/app/services/rag.py` (average of source similarity scores)
- [ ] T056 [P] [US4] Implement fallback response for low confidence (<0.7) in `rag.py` ("I don't have information on this topic")

### Frontend Confidence Display

- [ ] T057 [P] [US4] Add confidence badge to ChatMessage component (color-coded: green ≥0.8, yellow 0.7-0.8, red <0.7)
- [ ] T058 [P] [US4] Add sources section to ChatMessage component (display chapter, module, week, score)
- [ ] T059 [US4] Implement source navigation links (click chapter → navigate to textbook chapter URL)
- [ ] T060 [P] [US4] Add low confidence warning UI (display alert when confidence <0.7)

### Tests

- [ ] T061 [P] [US4] Test confidence calculation with `pytest chatbot-backend/tests/test_confidence.py`
- [ ] T062 [P] [US4] Test source display rendering with React Testing Library

**Parallel Execution Example (US4):**
```bash
# Parallel:
- T055, T056 (backend logic)
- T057, T058, T060 (frontend components)
- T061, T062 (tests)
```

**US4 Complete When:**
- [x] Confidence score visible in UI
- [x] Sources displayed with chapter names and scores
- [x] Low confidence triggers warning message
- [x] Manual test: Ask obscure question → confidence <0.7 → fallback message shown

---

## Phase 7: Polish & Cross-Cutting Concerns

**Goal:** Production readiness with styling, error handling, deployment, and documentation.

**Tasks:**

### Styling & UX

- [ ] T063 [P] Create CSS module in `physical-ai-textbook/src/components/ChatWidget/ChatWidget.module.css` (dark/light mode support, mobile responsive)
- [ ] T064 [P] Implement loading states in ChatWidget (animated dots while waiting for response)
- [ ] T065 [P] Implement error handling in ChatWidget (display user-friendly errors for API failures)

### Rate Limiting & Security

- [ ] T066 [P] Implement rate limiting middleware in `chatbot-backend/app/api/middleware/rate_limit.py` (100 req/hr per session ID)
- [ ] T067 [P] Implement input validation in chat endpoint (sanitize query, max length 500 chars)

### Deployment

- [ ] T068 Deploy backend to Railway with environment variables configured
- [ ] T069 Run Alembic migrations on Railway Postgres database
- [ ] T070 Build and deploy frontend to GitHub Pages with updated API URL
- [ ] T071 [P] Set up health check monitoring (Railway dashboard or external service)

### Documentation & Testing

- [ ] T072 [P] Generate OpenAPI documentation at `/api/docs` endpoint (automatic with FastAPI)
- [ ] T073 [P] Run full test suite and verify ≥80% coverage with `pytest --cov=app chatbot-backend/tests/`
- [ ] T074 [P] Run load test with 50 concurrent users using locust or k6
- [ ] T075 Create deployment guide in `chatbot-backend/DEPLOYMENT.md`

**Parallel Execution Example (Phase 7):**
```bash
# Parallel:
- T063, T064, T065, T066, T067 (independent improvements)
- T072, T073, T074 (documentation and testing)
# Sequential:
- T068 → T069 → T070 → T071 (deployment pipeline)
```

**Phase 7 Complete When:**
- [x] Dark/light mode styling applied
- [x] Rate limiting active (100 req/hr)
- [x] Backend deployed to Railway
- [x] Frontend deployed to GitHub Pages
- [x] Test coverage ≥80%
- [x] Load test passes (50 users, <3s response)

---

## Validation Checklist

### Format Validation

- [x] All tasks follow checklist format: `- [ ] [TaskID] [P?] [Story?] Description with file path`
- [x] Task IDs sequential (T001-T075)
- [x] [P] marker only on parallelizable tasks
- [x] [Story] label on user story phase tasks (US1, US2, US3, US4)
- [x] File paths specified in task descriptions

### Completeness Validation

- [x] Each user story has all needed tasks (models, services, endpoints, tests)
- [x] Each user story has independent test criteria
- [x] Dependencies clearly documented
- [x] Parallel execution examples provided per phase
- [x] MVP scope defined (US1 only)

### Coverage Validation

- [x] Setup phase (T001-T010): Project initialization
- [x] Foundational phase (T011-T020): Database, Qdrant, OpenAI setup
- [x] US1 phase (T021-T034): Core RAG pipeline
- [x] US2 phase (T035-T044): Text selection integration
- [x] US3 phase (T045-T054): Chat history persistence
- [x] US4 phase (T055-T062): Confidence and sources display
- [x] Polish phase (T063-T075): Deployment and optimization

---

## Progress Tracking

**Current Status:** Ready for implementation

### Task Progress (0/65 complete)

- [ ] Phase 1: Setup (0/10)
- [ ] Phase 2: Foundational (0/10)
- [ ] Phase 3: US1 (0/14)
- [ ] Phase 4: US2 (0/10)
- [ ] Phase 5: US3 (0/10)
- [ ] Phase 6: US4 (0/8)
- [ ] Phase 7: Polish (0/13)

### User Story Progress

- [ ] US1: Ask Questions (P1) - 0/14 tasks
- [ ] US2: Text Selection (P1) - 0/10 tasks
- [ ] US3: Chat History (P2) - 0/10 tasks
- [ ] US4: Confidence Display (P2) - 0/8 tasks

---

## Related Documentation

- **Spec:** `specs/2-rag-chatbot/spec.md`
- **Plan:** `specs/2-rag-chatbot/plan.md`
- **Data Model:** `specs/2-rag-chatbot/data-model.md`
- **API Schema:** `specs/2-rag-chatbot/contracts/api-schema.json`
- **Database Schema:** `specs/2-rag-chatbot/contracts/database-schema.sql`
- **Research:** `specs/2-rag-chatbot/research/research.md`
- **Constitution:** `.specify/memory/constitution-part2-rag-chatbot.md`

---

**Status:** Ready for Implementation
**Version:** 1.0.0
**Last Updated:** 2025-12-11
