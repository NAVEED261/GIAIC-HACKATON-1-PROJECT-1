---
feature: RAG Chatbot Integration
version: 1.0.0
status: Draft
created: 2025-12-11
updated: 2025-12-11
author: System
---

# Feature Specification: RAG Chatbot Integration

## Overview

This specification defines an intelligent Retrieval-Augmented Generation (RAG) chatbot system that integrates into the Physical AI Textbook to provide context-aware answers about course content. The chatbot combines vector search (Qdrant Cloud), language models (OpenAI), and conversational memory (Neon Postgres) to help students quickly find information, clarify concepts, and get personalized explanations based on textbook content.

The system consists of two primary components: a FastAPI backend providing RAG services and a React chat widget embedded in the Docusaurus frontend. The chatbot supports both direct questions and text selection queries, enabling students to ask questions about specific passages they're reading.

**Why it's needed:** Students spend significant time searching through documentation to find answers. An intelligent chatbot reduces friction by providing instant, context-aware answers directly within the learning environment, improving engagement and learning outcomes.

## Target Audience

**Primary:** Students using the Physical AI Textbook who need quick answers to questions about ROS 2, Isaac Sim, digital twins, and VLA concepts

**Secondary:**
- Instructors who want to understand common student questions and misconceptions
- Content creators who need feedback on which topics require clarification
- Self-directed learners who benefit from conversational learning interfaces

**Assumed Background:**
- Basic web browsing skills
- No technical knowledge required to use the chatbot
- Familiarity with the textbook structure (modules, chapters) is helpful but not required

## User Scenarios & Testing

### User Story 1: Ask Questions About Course Content (Priority: P1)

**As** a student learning Physical AI
**I need** to ask natural language questions about course topics and get accurate answers based on textbook content
**So that** I can quickly clarify concepts without manually searching through chapters

**Why this priority:** This is the core functionality that delivers immediate value to students. Without accurate, relevant answers, the chatbot provides no benefit over manual search.

**Independent Test:** Can be fully tested by sending various questions (ROS 2 concepts, Isaac Sim setup, hardware requirements) and verifying responses include accurate information with source citations from relevant chapters.

**Acceptance Scenarios:**

1. **Basic Question Answering**
   - **Given** I am reading about ROS 2 nodes
   - **When** I type "What is a ROS 2 node?" into the chatbot
   - **Then** I receive a concise answer (2-3 paragraphs) with citations to the relevant chapter (Module 1: ROS 2 Fundamentals)

2. **Multi-Topic Questions**
   - **Given** I need to understand integration between modules
   - **When** I ask "How do I use ROS 2 with Isaac Sim?"
   - **Then** I get an answer that references both Module 1 (ROS 2) and Module 3 (Isaac Sim) chapters

3. **Contextual Follow-Up**
   - **Given** I asked a question and received an answer
   - **When** I ask a related follow-up question in the same session
   - **Then** The chatbot maintains conversation context and provides coherent responses

4. **Source Attribution**
   - **Given** I receive an answer from the chatbot
   - **When** I review the response
   - **Then** I see source citations showing which chapters the information came from, with similarity scores

### User Story 2: Query Selected Text for Context-Aware Help (Priority: P1)

**As** a student reading a specific passage in the textbook
**I need** to select text and ask questions about that specific content
**So that** I can get clarification on confusing sections without having to rephrase the entire passage

**Why this priority:** Text selection queries provide targeted, context-aware assistance exactly when students encounter difficulty, significantly improving learning efficiency.

**Independent Test:** Can be fully tested by selecting text from any chapter, opening the chatbot (auto-opens with selection), and verifying the chatbot uses the selected text as additional context for answering questions.

**Acceptance Scenarios:**

1. **Text Selection Triggers Chatbot**
   - **Given** I am reading a chapter and encounter a confusing paragraph
   - **When** I select the text (>10 characters)
   - **Then** The chatbot widget auto-opens with a banner showing the selected text

2. **Context-Aware Responses**
   - **Given** I have text selected in the textbook
   - **When** I ask "What does this mean?"
   - **Then** The chatbot provides an explanation specific to the selected passage

3. **Clear Selected Context**
   - **Given** I asked a question about selected text
   - **When** I want to ask a general question
   - **Then** I can clear the selected text context by clicking an "×" button

4. **Selection Preservation**
   - **Given** I selected text and the chatbot opened
   - **When** I ask multiple questions about the same selection
   - **Then** The selected text context persists until I explicitly clear it

### User Story 3: Access Chat History Across Sessions (Priority: P2)

**As** a student using the textbook over multiple sessions
**I need** to see my previous chat conversations when I return
**So that** I can reference past answers and maintain learning continuity

**Why this priority:** Chat history supports incremental learning and reduces redundant questions, though students can still use the chatbot effectively without this feature.

**Independent Test:** Can be fully tested by asking questions in one session, closing the browser, reopening, and verifying previous conversations are visible.

**Acceptance Scenarios:**

1. **Session Persistence**
   - **Given** I asked several questions in a previous session
   - **When** I return to the textbook the next day
   - **Then** My chat history is preserved and visible in the chatbot

2. **Session Identification**
   - **Given** I am using the chatbot
   - **When** I check the browser's local storage
   - **Then** I see a unique session ID stored locally

3. **History Retrieval**
   - **Given** I have a session ID from a previous visit
   - **When** I reopen the chatbot
   - **Then** Past messages load asynchronously from the backend

4. **New Session Option**
   - **Given** I want to start fresh
   - **When** I clear my browser's local storage or use a new browser
   - **Then** A new session ID is generated and I see an empty chat history

### User Story 4: Understand Answer Confidence and Sources (Priority: P2)

**As** a student receiving answers from the chatbot
**I need** to see confidence scores and source citations for each answer
**So that** I can judge the reliability of information and explore source material

**Why this priority:** Transparency about answer quality helps students develop critical thinking and know when to consult original chapters for authoritative information.

**Independent Test:** Can be fully tested by asking various questions and verifying each response displays: (1) confidence score based on vector search similarity, (2) list of source chapters with relevance scores.

**Acceptance Scenarios:**

1. **Confidence Display**
   - **Given** I ask a question the chatbot can answer well
   - **When** I receive a response
   - **Then** I see a confidence score (e.g., "Confidence: 85%") based on vector search relevance

2. **Source Citations**
   - **Given** I receive an answer
   - **When** I review the response
   - **Then** I see a "Sources" section listing relevant chapters with similarity scores (e.g., "ROS 2 Fundamentals (92%)")

3. **Low Confidence Warning**
   - **Given** I ask a question about a topic not covered in the textbook
   - **When** The chatbot cannot find relevant context (confidence <70%)
   - **Then** I receive a message like "I don't have information on this topic in the Physical AI Textbook"

4. **Source Navigation**
   - **Given** I want to read the original content
   - **When** I click on a source citation
   - **Then** I am navigated to the relevant chapter in the textbook

## Edge Cases & Alternative Flows

### Edge Cases

1. **Ambiguous Questions**
   - **What happens when** a student asks an extremely vague question like "Tell me about robots"?
   - **Resolution:** Chatbot responds with a clarifying question: "The textbook covers several robot topics. Are you interested in ROS 2 basics (Module 1), simulation (Module 2), or Isaac Sim (Module 3)?"

2. **Out-of-Scope Questions**
   - **What happens when** a student asks about topics not in the textbook (e.g., "How do I get a robotics job")?
   - **Resolution:** Chatbot responds: "I don't have information on this topic in the Physical AI Textbook. I can help with ROS 2, digital twins, Isaac Sim, and VLA concepts."

3. **API Failures**
   - **What happens when** OpenAI API or Qdrant Cloud is unavailable?
   - **Resolution:** Chatbot displays error message: "Sorry, I'm temporarily unavailable. Please try again in a moment or use the textbook search." Frontend gracefully handles errors.

4. **Rate Limiting**
   - **What happens when** a user sends 100+ requests in an hour?
   - **Resolution:** Backend enforces rate limit (100 requests/hour per session), returns 429 status with message: "You've reached the hourly limit. Please try again in X minutes."

5. **Very Long Questions**
   - **What happens when** a user pastes an entire paragraph as a question (>500 characters)?
   - **Resolution:** Frontend truncates input at 500 characters with message: "Questions are limited to 500 characters. Please be more concise."

6. **Code Block Rendering**
   - **What happens when** the chatbot's answer includes code examples?
   - **Resolution:** Frontend renders code blocks with syntax highlighting using markdown parsing library.

### Alternative Flows

1. **Mobile Usage**
   - **How does** the chatbot work on mobile devices?
   - **Resolution:** Chat widget is responsive, occupying 90% of screen width on mobile with bottom-anchored input field for easy thumb typing.

2. **Text Selection on Mobile**
   - **How does** text selection work on touchscreens?
   - **Resolution:** Long-press to select text triggers chatbot auto-open, same as desktop mouse selection.

3. **Offline Usage**
   - **What happens when** a student loses internet connection?
   - **Resolution:** Chatbot displays offline indicator: "You're offline. The chatbot requires an internet connection." Static textbook content remains accessible.

4. **Multiple Tabs**
   - **What happens when** a user opens the textbook in multiple browser tabs?
   - **Resolution:** Each tab shares the same session ID (stored in localStorage), so chat history syncs across tabs after refresh.

## Requirements

### Functional Requirements

**FR-001:** System MUST provide a REST API endpoint (`POST /api/v1/chat`) that accepts a question, session ID, and optional selected text, returning an answer with sources and confidence score

**FR-002:** System MUST generate embeddings for user queries using OpenAI's `text-embedding-3-small` model (1536 dimensions)

**FR-003:** System MUST search for relevant textbook content using Qdrant Cloud vector database with cosine similarity, retrieving top 3-5 chunks with similarity threshold ≥0.7

**FR-004:** System MUST generate answers using OpenAI's `gpt-4o-mini` model with temperature=0.3, injecting retrieved context chunks as part of the prompt

**FR-005:** System MUST store chat history (user questions and assistant responses) in Neon Postgres database with fields: session_id, role (user/assistant), content, sources (JSONB), confidence, tokens_used, timestamp

**FR-006:** System MUST provide a REST API endpoint (`GET /api/v1/history/{session_id}`) that returns chat history for a given session, limited to last 50 messages

**FR-007:** Frontend MUST display a floating chat button (💬) fixed to bottom-right corner of all textbook pages

**FR-008:** Frontend MUST render a chat panel (400px × 600px on desktop, 90% screen width on mobile) when chat button is clicked, with header, message history, and input field

**FR-009:** Frontend MUST auto-open chat widget when user selects text >10 characters, displaying selected text in a dismissible banner

**FR-010:** Frontend MUST generate and persist a unique session ID in browser localStorage, reusing it across page refreshes

**FR-011:** Frontend MUST display loading indicator (animated dots) while waiting for API response

**FR-012:** Frontend MUST render chat messages in bubbles aligned left (assistant) and right (user), with source citations and confidence scores for assistant messages

**FR-013:** Backend MUST enforce rate limiting of 100 requests per hour per session ID

**FR-014:** Backend MUST include CORS headers allowing requests from GitHub Pages domain (`https://naveed261.github.io`)

**FR-015:** Backend MUST log all chat interactions (question, answer, sources, confidence, token usage) for analytics and improvement

**FR-016:** System MUST chunk textbook documents into 500-800 token segments with 100-token overlap before embedding

**FR-017:** System MUST extract metadata (chapter title, module number, week number) from document frontmatter and store in Qdrant as payload

**FR-018:** Backend MUST provide a health check endpoint (`GET /api/v1/health`) returning service status

**FR-019:** Frontend MUST support dark mode and light mode using Docusaurus CSS variables

**FR-020:** System MUST respond to queries within 3 seconds (vector search + LLM generation combined)

## Key Entities

### ChatSession
Represents a conversation thread with a user.

**Attributes:**
- id (UUID, primary key)
- session_id (string, unique identifier stored in client localStorage)
- created_at (timestamp)
- last_activity (timestamp)
- metadata (JSONB, stores user agent, client IP for analytics)

**Relationships:**
- Has many ChatMessage records

### ChatMessage
Represents a single message in a conversation.

**Attributes:**
- id (bigint, primary key)
- session_id (string, foreign key to ChatSession)
- role (enum: "user" or "assistant")
- content (text, the question or answer)
- sources (JSONB array, list of source chunks with chapter and score)
- confidence (float 0.0-1.0, average similarity score of retrieved chunks)
- tokens_used (integer, total tokens for OpenAI API call)
- created_at (timestamp)

**Relationships:**
- Belongs to one ChatSession

### TextbookChunk
Represents an embedded segment of textbook content stored in Qdrant.

**Attributes:**
- id (UUID, Qdrant point ID)
- text (string, the chunk content)
- embedding (vector, 1536-dimensional float array)
- chapter (string, chapter title from frontmatter)
- module (integer 1-4, module number)
- week (integer 1-13, week number)
- file_path (string, relative path to source markdown file)

**Relationships:**
- Derived from physical-ai-textbook markdown files

### ChatRequest
API request schema for asking questions.

**Attributes:**
- query (string, required, max 500 characters)
- session_id (string, required, client-generated UUID)
- selected_text (string, optional, max 2000 characters)

### ChatResponse
API response schema for answers.

**Attributes:**
- answer (string, LLM-generated response)
- sources (array of objects with chapter, score, module, week)
- session_id (string, echoed from request)
- confidence (float, average source similarity score)

## Success Criteria

This feature is complete when:

**SC-001:** Users can ask questions and receive relevant answers in under 3 seconds, with 90% of answers receiving confidence scores ≥0.7

**SC-002:** Chat widget is accessible from all textbook pages without breaking navigation or page layout

**SC-003:** Selected text (>10 characters) triggers chatbot auto-open with context preserved for question answering

**SC-004:** Chat history persists across browser sessions using localStorage session IDs and Neon Postgres backend storage

**SC-005:** 100% of API responses include source citations (chapter names) for transparency

**SC-006:** System gracefully handles API failures (OpenAI, Qdrant) with user-friendly error messages

**SC-007:** Backend passes all pytest tests with ≥80% code coverage (unit tests for API routes, RAG pipeline, database operations)

**SC-008:** Frontend chat widget is mobile-responsive and usable on screens from 375px to 1440px width

**SC-009:** Rate limiting (100 requests/hour) prevents abuse without impacting legitimate users

**SC-010:** All 70+ textbook markdown files are successfully embedded and searchable in Qdrant vector database

## Assumptions

1. **Textbook Content:** All textbook chapters have valid frontmatter metadata (title, module, week) required for source attribution

2. **API Costs:** OpenAI API usage stays within free tier limits during development; production deployment budgets $50/month for API costs based on estimated 5,000 queries/month

3. **Qdrant Storage:** 70+ documents with 500-800 token chunks fit within Qdrant Cloud free tier 1GB storage limit

4. **Database Storage:** Neon Postgres free tier (512MB storage) is sufficient for storing 6 months of chat history (~10,000 messages)

5. **Deployment:** Backend deploys to Railway or Vercel free tier with sufficient compute for 10-50 concurrent users

6. **Browser Support:** Users access textbook via modern browsers (Chrome, Firefox, Safari, Edge) with JavaScript enabled and localStorage support

7. **Session Management:** Client-side session ID generation is sufficient; server-side authentication is not required for Phase 1

8. **Embedding Quality:** OpenAI text-embedding-3-small provides sufficient semantic accuracy for educational content retrieval

9. **LLM Accuracy:** GPT-4o-mini with temperature=0.3 provides factually accurate answers when given relevant context chunks

10. **Network Latency:** Average API round-trip time (client → backend → OpenAI/Qdrant → backend → client) is under 3 seconds on broadband connections

## Open Questions

None. All requirements are specified with reasonable defaults based on industry standards for RAG chatbots and educational applications.

---

**Status:** Draft
**Version:** 1.0.0
**Last Updated:** 2025-12-11
