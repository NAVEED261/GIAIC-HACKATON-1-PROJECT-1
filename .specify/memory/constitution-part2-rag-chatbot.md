# RAG Chatbot Backend Constitution

**Project:** Physical AI Textbook - RAG Chatbot Integration (Part-2)
**Version:** 1.0.0
**Ratified:** 2025-12-11
**Last Amended:** 2025-12-11

## Core Principles

### I. API Design & Performance

**Mandate:** Backend API must be fast, reliable, and production-ready.

- Response time: < 3 seconds for RAG queries (vector search + LLM generation)
- Rate limiting: 100 requests/hour per user (prevent abuse)
- Error responses must be structured JSON with error codes and messages
- CORS properly configured for Docusaurus frontend
- Health check endpoint (`/health`) required for monitoring
- API documentation with OpenAPI/Swagger auto-generated
- Graceful degradation when external services fail (OpenAI, Qdrant)
- Connection pooling for database (Neon Postgres)

**Rationale:** Students expect instant responses. Slow or unreliable chatbot reduces trust and engagement.

---

### II. Vector Search Accuracy

**Mandate:** RAG pipeline must retrieve relevant context with high precision.

- Embedding model: `text-embedding-3-small` (OpenAI) for consistency
- Vector similarity threshold: ≥ 0.7 for context relevance
- Retrieve top 3-5 chunks per query (balance context vs token cost)
- Chunk size: 500-800 tokens with 100-token overlap for semantic continuity
- Metadata filtering: Support filtering by module, week, chapter
- Reranking: Implement score-based reranking of retrieved chunks
- Fallback response when no relevant context found (confidence < 0.7)
- Query preprocessing: Handle typos, expand abbreviations (ROS → Robot Operating System)

**Rationale:** Inaccurate context leads to hallucinated answers, undermining educational value.

---

### III. Data Security & Privacy

**Mandate:** User data must be protected with industry-standard practices.

- API keys stored in environment variables (`.env`), never committed to git
- User queries logged with anonymized IDs (no PII stored)
- Chat history encrypted at rest (Neon Postgres SSL mode)
- Input validation: Sanitize all user inputs to prevent SQL injection
- Rate limiting per IP address to prevent scraping
- OpenAI API key rotated quarterly
- Qdrant Cloud API key with read-only access for production
- No sensitive data (API keys, connection strings) in error messages

**Rationale:** Educational content is public, but user interactions deserve privacy protection.

---

### IV. Testing & Quality Assurance

**Mandate:** Comprehensive testing required before deployment.

- Unit tests: ≥ 80% code coverage (pytest)
- Integration tests: Test full RAG pipeline (query → vector search → LLM → response)
- API endpoint tests: Test all routes with valid/invalid inputs
- Load testing: Simulate 50 concurrent users (locust or k6)
- Embedding generation test: Verify all 70+ documents embedded successfully
- Database migration tests: Test schema changes with rollback
- Mock external services (OpenAI, Qdrant) in unit tests (cost efficiency)
- CI/CD pipeline runs all tests before deployment

**Test Coverage Requirements:**
- API routes: 100%
- RAG pipeline logic: ≥ 90%
- Database operations: ≥ 85%
- Utility functions: ≥ 75%

**Rationale:** Untested code in production leads to crashes and user frustration. High test coverage ensures reliability.

---

### V. Code Standards (Python/FastAPI)

**Mandate:** Maintain clean, readable, maintainable Python code.

- Follow PEP 8 style guide (enforced with `black` formatter)
- Type hints required for all functions (enforced with `mypy`)
- Docstrings for all public functions (Google style)
- Async/await for all I/O operations (database, API calls)
- Dependency injection for services (FastAPI Depends)
- Environment-based configuration (dev/staging/prod)
- Logging with structured format (JSON logs for production)
- Error handling: Catch specific exceptions, provide context

**File Structure:**
```
chatbot-backend/
├── app/
│   ├── api/          # FastAPI routes
│   ├── services/     # Business logic (RAG, embeddings)
│   ├── models/       # Pydantic models
│   ├── db/           # Database schemas and operations
│   └── core/         # Configuration, logging
├── tests/            # Pytest tests
├── skills/           # Subagent skill.md files
├── pyproject.toml    # Dependencies
└── .env.example      # Environment variable template
```

**Rationale:** Clean code is easier to debug, extend, and maintain. Type hints prevent runtime errors.

---

### VI. Integration Standards (Frontend-Backend)

**Mandate:** Seamless integration between Docusaurus frontend and FastAPI backend.

- RESTful API design (POST `/chat`, GET `/history/{session_id}`)
- JSON request/response format (standardized schemas)
- WebSocket support for streaming responses (optional, Phase 2)
- CORS headers configured for GitHub Pages domain
- Frontend error handling: Display user-friendly messages
- Loading states: Show spinner during API calls
- Text selection integration: Send selected text as context
- Session management: Generate unique session IDs client-side

**API Endpoints:**
- `POST /api/v1/chat` - Send query, get response
- `GET /api/v1/health` - Health check
- `GET /api/v1/history/{session_id}` - Retrieve chat history
- `POST /api/v1/feedback` - Collect user feedback (optional)

**Rationale:** Poor integration creates friction. Users should experience chatbot as native feature, not external tool.

---

### VII. Deployment & DevOps

**Mandate:** Automated deployment with zero-downtime updates.

- Backend deployed to Railway or Vercel (free tier)
- Environment variables managed via platform dashboard
- Database (Neon Postgres) with connection pooling
- Qdrant Cloud free tier (1GB storage, sufficient for 70+ documents)
- Health check monitoring (uptime alerts via BetterStack or UptimeRobot)
- Deployment rollback capability (keep last 3 versions)
- Staging environment for pre-production testing
- Production deployment triggered via GitHub Actions on `main` branch

**Environment Variables Required:**
```
OPENAI_API_KEY=sk-...
QDRANT_URL=https://...qdrant.io
QDRANT_API_KEY=...
DATABASE_URL=postgresql://...neon.tech
FRONTEND_URL=https://naveed261.github.io
```

**Rationale:** Manual deployment is error-prone. Automation ensures consistent, reliable releases.

---

### VIII. RAG Pipeline Quality

**Mandate:** LLM responses must be accurate, relevant, and educational.

- System prompt enforces educational tone and accuracy
- Context injection: Include retrieved chunks with source citations
- Answer format: Structured (summary + detailed explanation + code example if applicable)
- Hallucination prevention: Require LLM to cite sources from context
- Fallback response when no context found: "I don't have information on this topic in the textbook."
- Temperature: 0.3 (balance creativity vs factual accuracy)
- Max tokens: 500 (concise answers preferred)
- Source attribution: Display chapter/module for each answer

**System Prompt Template:**
```
You are a helpful teaching assistant for the Physical AI Textbook course.
Answer questions based ONLY on the provided context from the textbook.
If the context doesn't contain the answer, say "I don't have information on this topic."
Cite the chapter/module for each answer.
Provide code examples when relevant.
```

**Rationale:** RAG quality directly impacts learning outcomes. Hallucinated answers mislead students.

---

## Governance

### Amendment Process

1. Propose amendment with rationale and impact assessment
2. Review by backend developer and curriculum designer
3. Approval requires consensus
4. Document in ADR (Architecture Decision Record)
5. Update constitution version (MAJOR for breaking changes, MINOR for additions)
6. Communicate changes to all contributors

### Enforcement

- All pull requests must pass automated quality gates (tests, type checking, linting)
- Code reviews verify compliance with constitution principles
- Non-compliance blocks merge until resolved
- Exceptions require explicit justification documented in PR description

### Deviation Policy

- Deviations from constitution require documented justification
- Temporary deviations allowed for prototyping; must be resolved before production
- Permanent deviations require amendment process (see above)

---

**Version:** 1.0.0
**Ratified:** 2025-12-11
**Last Amended:** 2025-12-11
**Next Review:** 2026-03-11 (Quarterly review cycle)
