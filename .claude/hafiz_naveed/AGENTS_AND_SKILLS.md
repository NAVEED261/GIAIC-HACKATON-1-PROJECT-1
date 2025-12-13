# 🤖 COMPLETE AGENTS GUIDE - DETAILED BREAKDOWN

## 📊 TOTAL AGENTS: 16

**GROUP 1: Domain Experts (5)**
**GROUP 2: Workflow Commands (11)**

---

# 🔴 GROUP 1: DOMAIN-SPECIFIC AGENTS (5)

---

## **AGENT #1: DOCUSAURUS CHATBOT AGENT**

### **Command:** `/docusaurus-chatbot-agent`

### **Skills (Roman Urdu):**
- React 18 + TypeScript - React library mein components banate ho
- Docusaurus v3 integration - Docusaurus website mein chatbot add karta hai
- Text selection API - User jab text select kare to us pe action le sakte ho
- Fetch API - Frontend se backend ko request bhejna
- CSS modules - Component-specific styling (CSS ko isolated rakhna)
- Dark/light mode - Raat ka mode aur din ka mode
- Mobile responsive design - Chhoti screen aur badi screen dono mein chalega
- State management - React hooks se data manage karna (useState, useEffect)
- Error handling - Jab error aye to kya show karna

### **PRACTICAL USE CASE:**

**User Query:** "Yaar, mara chatbot widget bahut slow open hota hai aur dark mode bhi chahiye"

### **Step-by-Step Process:**

```
STEP 1: USER QUERY
├─ User: "Chatbot widget ko fast karna hai aur dark mode add karna hai"

STEP 2: AGENT ACTIVATION
├─ /docusaurus-chatbot-agent activate ho jaata hai

STEP 3: CODE ANALYSIS
├─ Agent: "Aapka ChatWidget.tsx file dekh raha hoon..."
├─ Agent: "Isme useState aur useEffect ho rahe hain"
├─ Agent: "Opening animation 500ms ka hai"
├─ Agent: "Dark mode abhi nahi hai"

STEP 4: PERFORMANCE CHECK
├─ Agent: "Performance issues:"
├─   - API call har bar naye kiye jaa rahe hain (caching nahi ho)
├─   - CSS animation heavy hai
├─   - Message rendering optimize nahi hai

STEP 5: SOLUTIONS SUGGEST
├─ Agent: "Theek hai, maine 3 solutions diye hain:"
├─   1. useState mein useMemo add karo
├─   2. CSS animation ko hardware-accelerated banao
├─   3. Dark mode ke liye CSS variables use karo

STEP 6: CODE EXAMPLES DENA
├─ Agent: "Yeh raha updated ChatWidget.tsx code:"
├─
├─ ```typescript
├─ import React, { useState, useMemo } from 'react';
├─ import styles from './styles.module.css';
├─
├─ const ChatWidget = () => {
├─   const [isDarkMode, setIsDarkMode] = useState(false);
├─   const [isOpen, setIsOpen] = useState(false);
├─
├─   // Memoized messages - performance boost
├─   const memoizedMessages = useMemo(() => messages, [messages]);
├─
├─   return (
├─     <div className={`${styles.widget} ${isDarkMode ? styles.dark : ''}`}>
├─       {/* Dark mode toggle */}
├─       <button onClick={() => setIsDarkMode(!isDarkMode)}>
├─         {isDarkMode ? '☀️' : '🌙'}
├─       </button>
├─
├─       {/* Optimized rendering */}
├─       {memoizedMessages.map(msg => (
├─         <div key={msg.id} className={styles.message}>
├─           {msg.content}
├─         </div>
├─       ))}
├─     </div>
├─   );
├─ };
├─ ```

STEP 7: CSS UPDATES
├─ Agent: "Yeh raha optimized CSS:"
├─
├─ `.widget {
├─   animation: slideIn 300ms ease-out; /* Fast animation */
├─   --bg-color: #fff;
├─   --text-color: #000;
├─   background-color: var(--bg-color);
├─   color: var(--text-color);
├─ }
├─
├─ .widget.dark {
├─   --bg-color: #1a1a1a;
├─   --text-color: #fff;
├─ }
├─
├─ @keyframes slideIn {
├─   from { transform: translateY(10px); opacity: 0; }
├─   to { transform: translateY(0); opacity: 1; }
├─ }`

STEP 8: TESTING GUIDE
├─ Agent: "Testing steps:"
├─   1. Browser DevTools > Lighthouse > Performance check karo
├─   2. Dark mode toggle test karo (kaam karega ab)
├─   3. Mobile responsive test karo (375px width mein)
├─   4. Backend se real query test karo

STEP 9: FINAL CHECKLIST
├─ Agent: "Yeh check kar lo pehle:"
├─   ✅ Opening animation < 300ms
├─   ✅ Dark mode CSS variables working
├─   ✅ Mobile responsive (tested)
├─   ✅ Backend calls working
├─   ✅ Error handling present
```

### **AGENT'S FINAL REPLY:**

"Bilkul! Maine 2 issues fix kar diye:

**Performance:** Animation ko hardware-accelerated banaya, useState mein useMemo add kiya. Ab opening time 500ms se 300ms ho gaya.

**Dark Mode:** CSS variables use kiye hain. User 🌙 button click karega to automatically dark theme apply hoga.

Code upar dikhai hai, bas copy-paste karo aur test karo. Agar aur improve karna ho to batana! 🚀"

---

---

## **AGENT #2: FASTAPI AGENT**

### **Command:** `/fastapi-agent`

### **Skills (Roman Urdu):**
- FastAPI 0.104+ framework - Python mein web server banate ho
- Async/await patterns - Non-blocking code likho (ek sath multiple requests chalenge)
- Pydantic v2 validation - Data input validation (kya data sahi hai ya galat)
- Dependency injection - Code reuse ke liye dependencies share karna
- CORS middleware - Frontend ko backend se data lene ki permission
- Error handling - 404, 500, validation errors handle karna
- Structured logging - Kya kya ho raha hai track karna (logs mein)

### **PRACTICAL USE CASE:**

**User Query:** "Bhai, mara API timeout ho raha hai kabhi kabhi aur 500 error arha hai. Kya kara?"

### **Step-by-Step Process:**

```
STEP 1: USER QUERY
├─ User: "FastAPI backend mein kharab lag raha hai, timeout hota hai"

STEP 2: AGENT ACTIVATION
├─ /fastapi-agent activate ho jaata hai

STEP 3: ISSUE DIAGNOSIS
├─ Agent: "Thik hai, main analyze karta hoon:"
├─ Agent: "500 errors usually 3 chezo se aate hain:"
├─   1. Database query slow ho
├─   2. OpenAI API call hanging ho
├─   3. Vector search timeout ho

STEP 4: CODE REVIEW
├─ Agent: "Mujhe aapka app/main.py aur services/ dikha rahe ho"
├─ Agent: "Dekha! Problem yeh hai:"
├─   - RAG service mein no timeout set hai
├─   - Database queries optimized nahi hain
├─   - Error handling incomplete hai

STEP 5: SOLUTIONS PROVIDE
├─ Agent: "Maine 3 fixes identify kiye:"
├─   1. Timeouts add karna (FastAPI level)
├─   2. Database queries optimize karna
├─   3. Better error handling add karna

STEP 6: CODE EXAMPLE - TIMEOUT FIX
├─ Agent: "Yeh raha app/main.py update:"
├─
├─ ```python
├─ from fastapi import FastAPI, Request
├─ from contextlib import asynccontextmanager
├─ import asyncio
├─
├─ # Timeout manager
├─ @asynccontextmanager
├─ async def lifespan(app: FastAPI):
├─     yield
├─     # Cleanup code if needed
├─
├─ app = FastAPI(lifespan=lifespan)
├─
├─ @app.post("/api/v1/chat")
├─ async def chat(request: ChatRequest):
├─     try:
├─         # 10 second timeout set karo
├─         result = await asyncio.wait_for(
├─             rag_service.process_query(request.query),
├─             timeout=10.0
├─         )
├─         return result
├─     except asyncio.TimeoutError:
├─         return {"error": "Request timeout - try shorter query"}
├─ ```

STEP 7: CODE EXAMPLE - OPTIMIZATION
├─ Agent: "Database queries optimize karna:"
├─
├─ ```python
├─ # app/services/rag_service.py - BEFORE (slow)
├─ async def process_query(self, query: str):
├─     # Har bar full DB scan
├─     chunks = await db.query("SELECT * FROM chunks")
├─
├─ # AFTER (fast - with indexing)
├─ async def process_query(self, query: str):
├─     # Vector search mein limiting
├─     query_vector = await self.embedding_service.embed(query)
├─
├─     results = await self.qdrant.search(
├─         vector=query_vector,
├─         limit=5,  # Top 5 only
├─         score_threshold=0.3  # Low threshold
├─     )
├─
├─     return results  # Direct return, no extra queries
├─ ```

STEP 8: LOGGING ADD KARO
├─ Agent: "Kya problem hai track karne ke liye logging add karo:"
├─
├─ ```python
├─ import logging
├─ import time
├─
├─ logger = logging.getLogger(__name__)
├─
├─ @app.post("/api/v1/chat")
├─ async def chat(request: ChatRequest):
├─     start_time = time.time()
├─
├─     try:
├─         logger.info(f"Query received: {request.query}")
├─
├─         result = await asyncio.wait_for(
├─             rag_service.process_query(request.query),
├─             timeout=10.0
├─         )
├─
├─         elapsed = time.time() - start_time
├─         logger.info(f"Query processed in {elapsed:.2f}s")
├─
├─         return result
├─     except asyncio.TimeoutError:
├─         logger.error("Request timeout after 10s")
├─         return {"error": "Timeout"}
├─ ```

STEP 9: TESTING
├─ Agent: "Test karne ke liye ye steps:"
├─   1. Local pe backend start karo
├─   2. Normal query bhejo (response time dekho)
├─   3. Slow query bhejo (timeout test karo)
├─   4. Logs check karo terminal mein

STEP 10: MONITORING
├─ Agent: "Production mein Render logs check karo:"
├─   1. Render dashboard > Logs
├─   2. Performance metrics dekho
├─   3. Agar still slow ho to database ke queries check karo
```

### **AGENT'S FINAL REPLY:**

"Aha! Maine issue find kar liya:

**Problem 1:** Timeouts nahi the set. OpenAI API kabhi hang ho jaata tha.
**Solution:** FastAPI ke asyncio.wait_for() se 10-second timeout add kiya.

**Problem 2:** Database queries har baar full scan ho rahe the.
**Solution:** Vector search mein limit=5 aur score_threshold set kiya.

**Problem 3:** Error handling nahi tha.
**Solution:** Try-except blocks add kiye aur logging set up kiya.

Ab test karo, response time pata chal jaega logs mein. Agar aur problem ho to bata! 🚀"

---

---

## **AGENT #3: QDRANT AGENT**

### **Command:** `/qdrant-agent`

### **Skills (Roman Urdu):**
- Qdrant vector database - Document search ke liye (similarity search)
- Document ingestion - Files ko vectors mein convert karna
- Vector embeddings - 1536-dimensional vectors (OpenAI model se)
- Cosine similarity - Kaunse document similar hain find karna
- Payload management - Extra metadata store karna (chapter name, week, etc)
- Collection management - Database structure create/manage karna
- Score filtering - Confidence threshold set karna (relevant results filter karna)

### **PRACTICAL USE CASE:**

**User Query:** "Yaar, search results bilkul galat aarhe. User ne 'ROS 2 nodes' poochha to humanoid robotics ka answer arha hai"

### **Step-by-Step Process:**

```
STEP 1: USER QUERY
├─ User: "Search results wrong hain, irrelevant documents aa rahe hain"

STEP 2: AGENT ACTIVATION
├─ /qdrant-agent activate ho jaata hai

STEP 3: ISSUE DIAGNOSIS
├─ Agent: "Search results galat aane ke reasons:"
├─   1. Embeddings galat generated ho rahe hain
├─   2. Confidence threshold galat set hai
├─   3. Documents properly indexed nahi hain

STEP 4: VECTOR DB CHECK
├─ Agent: "Qdrant collection check kar raha hoon:"
├─   - Collection name: "physical_ai_docs"
├─   - Vector size: 1536 ✓ (Correct)
├─   - Documents: 54 ✓
├─   - BUT: Confidence threshold 0.7 set hai (bahut high!)

STEP 5: ROOT CAUSE FOUND
├─ Agent: "Problem mila! MIN_CONFIDENCE_THRESHOLD 0.7 hai"
├─ Agent: "Matlab sirf 70% se zyada similar docs dikh rahe hain"
├─ Agent: "ROS 2 search karte hue humanoid result 65% similar tha"
├─ Agent: "0.7 se kam tha, isliye filter out ho gaya"

STEP 6: SOLUTION - THRESHOLD LOWER KARNA
├─ Agent: "Yeh fix karo app/core/config.py mein:"
├─
├─ ```python
├─ # BEFORE (too strict)
├─ MIN_CONFIDENCE_THRESHOLD: float = 0.7
├─
├─ # AFTER (better recall)
├─ MIN_CONFIDENCE_THRESHOLD: float = 0.3
├─ ```

STEP 7: SOLUTION - BETTER FILTERING
├─ Agent: "Pehle filter karo, phir sort karo:"
├─
├─ ```python
├─ # app/services/qdrant_service.py
├─
├─ async def search(self, query_vector, limit, score_threshold):
├─     results = await self.client.search(
├─         collection_name="physical_ai_docs",
├─         query_vector=query_vector,
├─         limit=limit,
├─         score_threshold=score_threshold  # Now 0.3
├─     )
├─
├─     # Results automatically sorted by score (highest first)
├─     # Top 5 mil jaayenge jo similar hain
├─
├─     return [
├─         {
├─             "chapter": hit.payload["chapter"],
├─             "content": hit.payload["content"],
├─             "score": hit.score,  # 0.95 = 95% similar
├─             "week": hit.payload["week"],
├─             "module": hit.payload["module"]
├─         }
├─         for hit in results
├─     ]
├─ ```

STEP 8: CHECK INGESTION
├─ Agent: "Dekho collection mein sab documents sahi hain ya nahi:"
├─
├─ ```python
├─ # Run this to check
├─ async def check_collection():
├─     collection = await qdrant.get_collection("physical_ai_docs")
├─     print(f"Total vectors: {collection.points_count}")
├─     print(f"Vector size: {collection.config.params.vectors.size}")
├─
├─     # Sample queries test karo
├─     test_query = "ROS 2 nodes"
├─     embedding = await embedding_service.embed(test_query)
├─     results = await qdrant.search(
├─         collection_name="physical_ai_docs",
├─         query_vector=embedding,
├─         limit=5,
├─         score_threshold=0.3
├─     )
├─
├─     for hit in results:
├─         print(f"{hit.payload['chapter']}: {hit.score:.2f}")
├─ ```

STEP 9: DEBUGGING
├─ Agent: "Agar aur bhi specific results chahiye:"
├─
├─ ```python
├─ # Filter by module
├─ results = await qdrant.search(
├─     collection_name="physical_ai_docs",
├─     query_vector=query_embedding,
├─     query_filter=Filter(
├─         must=[
├─             FieldCondition(
├─                 key="module",
├─                 match=MatchValue(value=1)  # Module 1 only
├─             )
├─         ]
├─     ),
├─     limit=5,
├─     score_threshold=0.3
├─ )
├─ ```

STEP 10: VALIDATION
├─ Agent: "Test karo yeh queries:"
├─   1. "ROS 2 nodes" → Should get ros2-fundamentals (0.95+)
├─   2. "Digital twins" → Should get digital-twins.md (0.90+)
├─   3. "Isaac Sim" → Should get nvidia-isaac-sim.md (0.88+)
├─   4. "Random words xyz" → Should get fallback (0.25-0.35)
```

### **AGENT'S FINAL REPLY:**

"Bilkul! Issue pata chal gaya:

**Problem:** MIN_CONFIDENCE_THRESHOLD 0.7 set tha (70% similarity chahiye)
ROS 2 ke relevant results 0.65-0.68 score par tha, filter out ho rahe the.

**Fix:** Threshold 0.7 se 0.3 kiya (30% minimum)
Ab 'ROS 2 nodes' query ke liye top 5 relevant results aayenge.

**Update karo:**
- `app/core/config.py`: MIN_CONFIDENCE_THRESHOLD = 0.3
- Backend restart karo
- Test query bhejo

Ab sahi results ayenge! 🎯"

---

---

## **AGENT #4: OPENAI AGENT**

### **Command:** `/openai-agent`

### **Skills (Roman Urdu):**
- OpenAI API integration - API calls karna
- text-embedding-3-small model - Text ko 1536-dimensional vectors mein convert karna
- gpt-4o-mini model - Fast aur cheap ChatGPT API
- System prompt engineering - AI ko instructions dena (kya batana hai, kaise batana hai)
- Context injection - Search results ko prompt mein add karna
- Token management - Cost control (tokens consumption track karna)
- Rate limiting - API limits within rehna
- Error handling - API failures handle karna

### **PRACTICAL USE CASE:**

**User Query:** "Chatbot sirf 'I'm sorry' reply de raha hai. Proper answer nahi de raha aur confidence bhi nahi hai"

### **Step-by-Step Process:**

```
STEP 1: USER QUERY
├─ User: "Chatbot ka response bilkul galat hai, har query pe 'I'm sorry' arha hai"

STEP 2: AGENT ACTIVATION
├─ /openai-agent activate ho jaata hai

STEP 3: ISSUE DIAGNOSIS
├─ Agent: "This happens when:"
├─   1. System prompt apologie karta hai
├─   2. Context nahi milta (vector search fail)
├─   3. API response khrab hai
├─   4. Confidence low hai

STEP 4: SYSTEM PROMPT CHECK
├─ Agent: "Aapka system prompt dekh raha hoon"
├─ Agent: "AHA! Problem mila:"
├─
├─ CURRENT (WRONG):
├─ "If you don't have relevant information, apologize and say sorry..."
├─
├─ CHANGE TO:
├─ "Answer confidently based on the course content..."

STEP 5: CONTEXT ISSUE
├─ Agent: "Agar vector search empty aye, to fallback mode chalani chahiye"
├─ Agent: "Aapka RAG pipeline:"
├─   1. Query embedding generate karta hai
├─   2. Qdrant search karta hai
├─   3. Agar 0 results: "I'm sorry" message deta hai
├─   4. GALAT! Should give best answer from general knowledge

STEP 6: UPDATED SYSTEM PROMPT
├─ Agent: "Yeh raha fixed system prompt:"
├─
├─ ```python
├─ # app/services/chat_service.py
├─
├─ def _create_system_prompt(self, context: str) -> str:
├─     return f"""You are an expert Physical AI teaching assistant.
├─
├─ **Your Task**: Answer student questions confidently.
├─
├─ **Guidelines**:
├─ 1. Answer DIRECTLY using the context below
├─ 2. Explain concepts clearly
├─ 3. Use examples from context when available
├─ 4. **DO NOT apologize** - provide helpful answers
├─ 5. If context limited, use your knowledge of the topic
├─
├─ **Course Content:**
├─ {context}
├─
├─ Answer the question clearly and helpfully."""
├─ ```

STEP 7: RAG SERVICE UPDATE
├─ Agent: "RAG pipeline mein fallback mode add karo:"
├─
├─ ```python
├─ # app/services/rag_service.py
├─
├─ async def process_query(self, query: str):
├─     # Step 1: Try vector search
├─     search_results = []
├─     context = ""
├─
├─     try:
├─         query_embedding = await self.embedding_service.embed(query)
├─         search_results = await self.qdrant.search(
├─             query_vector=query_embedding,
├─             limit=5,
├─             score_threshold=0.3
├─         )
├─
├─         context = "\n\n".join([
├─             f"[{r['chapter']}]\n{r['content']}"
├─             for r in search_results
├─         ])
├─     except Exception as e:
├─         logger.warning(f"Search failed: {e}")
├─         context = ""  # Empty context, no apology!
├─
├─     # Step 2: Generate response (with or without context)
├─     answer = await self.chat_service.generate_response(
├─         query=query,
├─         context=context
├─     )
├─
├─     # Step 3: Calculate confidence
├─     confidence = max([r["score"] for r in search_results]) if search_results else 0.5
├─
├─     return {
├─         "answer": answer,
├─         "sources": [{"chapter": r["chapter"], "score": r["score"]} for r in search_results],
├─         "confidence": confidence
├─     }
├─ ```

STEP 8: TOKEN OPTIMIZATION
├─ Agent: "OpenAI API cost control:"
├─
├─ ```python
├─ import tiktoken
├─
├─ def count_tokens(text: str) -> int:
├─     encoding = tiktoken.encoding_for_model("gpt-4o-mini")
├─     return len(encoding.encode(text))
├─
├─ # Agar context bahut bada hai to trim karo
├─ if count_tokens(context) > 1000:
├─     context = context[:1000] + "..."
├─ ```

STEP 9: ERROR HANDLING
├─ Agent: "API failures handle karo:"
├─
├─ ```python
├─ try:
├─     response = await self.client.chat.completions.create(
├─         model="gpt-4o-mini",
├─         messages=[...],
├─         timeout=10.0
├─     )
├─ except openai.RateLimitError:
├─     return "API rate limit exceeded, try again later"
├─ except openai.APIError as e:
├─     logger.error(f"OpenAI error: {e}")
├─     return "Service temporarily unavailable"
├─ ```

STEP 10: TESTING
├─ Agent: "Test these scenarios:"
├─   1. Normal query with good context → Should give confident answer
├─   2. Query with bad context → Should still answer, but low confidence
├─   3. Nonsense query → Should give best effort answer
├─   4. API failure → Should handle gracefully
```

### **AGENT'S FINAL REPLY:**

"Bilkul samajh agaya!

**Problem 1:** System prompt apologetic tha
**Fix:** System prompt change kiya, ab confident answer dega

**Problem 2:** Empty context aye to 'I'm sorry' deta tha
**Fix:** Fallback mode add kiya, general knowledge use karega

**Problem 3:** Token optimization nahi tha
**Fix:** tiktoken se token counting add ki

**Changes:**
1. Update `app/services/chat_service.py` - system prompt
2. Update `app/services/rag_service.py` - fallback mode
3. Restart backend
4. Test query bhejo

Ab confident aur helpful answers aayenge! 💪"

---

---

## **AGENT #5: NEON POSTGRES AGENT**

### **Command:** `/neon-postgres-agent`

### **Skills (Roman Urdu):**
- Neon serverless Postgres - Cloud database (free tier)
- AsyncPG driver - Async database connections (ek sath multiple queries)
- SQLAlchemy 2.0 ORM - Database operations ko Python se manage karna
- Connection pooling - Multiple connections share karna
- Query optimization - Queries ko fast banano
- Schema migrations - Database structure change karna (Alembic se)
- Session management - Chat history save karna

### **PRACTICAL USE CASE:**

**User Query:** "Chat history save nahi ho rahi database mein. User ko pichla conversation nahi dikhta"

### **Step-by-Step Process:**

```
STEP 1: USER QUERY
├─ User: "Database mein chat history nahi save ho rahi"

STEP 2: AGENT ACTIVATION
├─ /neon-postgres-agent activate ho jaata hai

STEP 3: ISSUE DIAGNOSIS
├─ Agent: "Chat history issues usually:"
├─   1. Database connection fail
├─   2. Schema not created
├─   3. SQLAlchemy models galat
├─   4. Async operations properly nahi setup

STEP 4: CONNECTION CHECK
├─ Agent: "DATABASE_URL check kar raha hoon:"
├─ Agent: "Found issue!"
├─
├─ WRONG: postgresql://...
├─ RIGHT: postgresql+asyncpg://...
├─
├─ Aapne postgresql driver set kiya hai (sync)
├─ Zaroor asyncpg (async) chahiye

STEP 5: CONNECTION FIX
├─ Agent: "Update .env file:"
├─
├─ BEFORE:
├─ DATABASE_URL=postgresql://user:password@host/db
├─
├─ AFTER:
├─ DATABASE_URL=postgresql+asyncpg://user:password@host/db

STEP 6: SQLALCHEMY MODELS CREATE
├─ Agent: "Models banao app/models/chat.py mein:"
├─
├─ ```python
├─ from sqlalchemy import Column, String, DateTime, Integer
├─ from sqlalchemy.ext.declarative import declarative_base
├─ from datetime import datetime
├─
├─ Base = declarative_base()
├─
├─ class ChatSession(Base):
├─     __tablename__ = "chat_sessions"
├─
├─     session_id = Column(String, primary_key=True)
├─     user_id = Column(String, nullable=True)
├─     created_at = Column(DateTime, default=datetime.utcnow)
├─     updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
├─
├─ class ChatMessage(Base):
├─     __tablename__ = "chat_messages"
├─
├─     id = Column(Integer, primary_key=True)
├─     session_id = Column(String, nullable=False)
├─     role = Column(String)  # 'user' or 'assistant'
├─     content = Column(String)
├─     confidence = Column(float, nullable=True)
├─     created_at = Column(DateTime, default=datetime.utcnow)
├─ ```

STEP 7: DATABASE SERVICE
├─ Agent: "Service layer banao app/services/database_service.py:"
├─
├─ ```python
├─ from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
├─ from app.models.chat import ChatSession, ChatMessage
├─
├─ engine = create_async_engine(
├─     settings.DATABASE_URL,
├─     echo=False,
├─     pool_size=5,  # Connection pooling
├─     max_overflow=10
├─ )
├─
├─ async def save_message(session_id: str, role: str, content: str):
├─     async with AsyncSession(engine) as session:
├─         # Pehle session check karo, nahi to create karo
├─         existing = await session.execute(
├─             select(ChatSession).where(ChatSession.session_id == session_id)
├─         )
├─         if not existing.scalar():
├─             session.add(ChatSession(session_id=session_id))
├─             await session.commit()
├─
├─         # Message save karo
├─         message = ChatMessage(
├─             session_id=session_id,
├─             role=role,
├─             content=content
├─         )
├─         session.add(message)
├─         await session.commit()
├─
├─ async def get_history(session_id: str):
├─     async with AsyncSession(engine) as session:
├─         result = await session.execute(
├─             select(ChatMessage)
├─             .where(ChatMessage.session_id == session_id)
├─             .order_by(ChatMessage.created_at)
├─         )
├─         return result.scalars().all()
├─ ```

STEP 8: ALEMBIC MIGRATIONS
├─ Agent: "Database schema create karo:"
├─
├─ ```bash
├─ # Alembic init karo (agar nahi kiya)
├─ alembic init alembic
├─
├─ # Migration generate karo
├─ alembic revision --autogenerate -m "Add chat tables"
├─
├─ # Apply migration
├─ alembic upgrade head
├─ ```

STEP 9: CHAT ENDPOINT UPDATE
├─ Agent: "Chat endpoint mein history save karo:"
├─
├─ ```python
├─ @router.post("/api/v1/chat")
├─ async def chat(request: ChatRequest, db_service: DatabaseService):
├─     # Save user message
├─     await db_service.save_message(
├─         session_id=request.session_id,
├─         role="user",
├─         content=request.query
├─     )
├─
├─     # Get response
├─     result = await rag_service.process_query(request.query)
├─
├─     # Save assistant message
├─     await db_service.save_message(
├─         session_id=request.session_id,
├─         role="assistant",
├─         content=result["answer"]
├─     )
├─
├─     return result
├─ ```

STEP 10: TESTING
├─ Agent: "Test karo yeh steps:"
├─   1. Backend restart karo
├─   2. Query bhejo (message save hona chahiye)
├─   3. Neon dashboard check karo (chat_messages table mein entry dekho)
├─   4. Same session_id se phir query bhejo (history show kare)
```

### **AGENT'S FINAL REPLY:**

"Bilkul! Chat history issue fix:

**Problem 1:** DATABASE_URL mein postgresql use tha (sync driver)
**Fix:** postgresql+asyncpg se change kiya (async)

**Problem 2:** Database tables nahi the
**Fix:** SQLAlchemy models create kiye (ChatSession, ChatMessage)

**Problem 3:** Save logic nahi tha
**Fix:** Database service banaya aur chat endpoint mein integrate kiya

**Steps to implement:**
1. Update .env: DATABASE_URL=postgresql+asyncpg://...
2. Create models (app/models/chat.py)
3. Create database service (app/services/database_service.py)
4. Setup Alembic migrations
5. Restart backend
6. Test query - history save hona chahiye

Ab chat history properly save hogi! 📝"

---

---

# 🟢 GROUP 2: SPECKIT PLUS WORKFLOW AGENTS (11)

---

## **AGENT #6: /sp.specify**

### **Purpose:** Feature specification banao

### **PRACTICAL USE CASE:**

**User Query:** "Mujha ek new feature specification chahiye: 'User feedback system' banao"

### **Step-by-Step Process:**

```
STEP 1: USER REQUEST
├─ User: "/sp.specify - User feedback system banao"

STEP 2: AGENT ACTIVATION
├─ /sp.specify command activate ho jaata hai

STEP 3: REQUIREMENTS GATHERING
├─ Agent: "Thik hai, maine 5 targeted clarification questions poocha:"
├─
├─ Q1: "Feedback kaun collect karega? (Students, Admins, Both?)"
├─ Q2: "Feedback kaun kaun sa fields hoga? (Rating, Text, Category?)"
├─ Q3: "Database mein long-term rakha jayega ya delete hoga?"
├─ Q4: "Real-time notification chahiye feedback par?"
├─ Q5: "Analytics dashboard bhe chahiye?"

STEP 4: USER ANSWERS
├─ User: "Students se feedback, 5-star rating + comment, store forever, no notification, yes analytics"

STEP 5: SPECIFICATION DOCUMENT GENERATE
├─ Agent: "Ab full specification banata hoon..."
├─
├─ Output file: specs/feedback-system/spec.md
├─
├─ Content:
├─ ├─ Feature Overview
├─ ├─ User Stories
├─ │  ├─ US1: Student submit feedback
├─ │  ├─ US2: Admin view analytics
├─ │  └─ US3: Generate reports
├─ ├─ Functional Requirements (FR-001 to FR-010)
├─ ├─ Non-Functional Requirements
├─ ├─ Success Criteria (SC-001 to SC-008)
├─ ├─ Data Models
├─ └─ Test Cases

STEP 6: SUCCESS METRICS
├─ Agent: "Success ko measure karega in se:"
├─   ✅ 95% feedback submission success rate
├─   ✅ Analytics dashboard loads in < 2s
├─   ✅ 5-star rating system working
├─   ✅ Comments searchable

STEP 7: OUTPUT
├─ Agent creates:
├─   1. specs/feedback-system/spec.md (200+ lines)
├─   2. Creates feature branch: feedback-system
├─   3. Validates against constitution
```

### **AGENT'S FINAL REPLY:**

"✅ Specification tiyar ho gaya!

**File:** specs/feedback-system/spec.md
**Branch:** feedback-system (created)

**Contains:**
- 3 User Stories
- 10 Functional Requirements
- 8 Success Criteria
- Data models
- Test cases

Next step: `/sp.plan` chalao implementation plan ke liye! 📋"

---

## **AGENT #7: /sp.clarify**

### **Purpose:** Ambiguous requirements ko clear karo

### **PRACTICAL USE CASE:**

**User Query:** "/sp.clarify - Spec mein confusion hai"

### **Step-by-Step Process:**

```
STEP 1: USER REQUEST
├─ User: "Kuch points samjha nahi aaye specification mein"

STEP 2: ANALYSIS
├─ Agent: Spec.md padhta hai
├─ Agent: Ambiguous sections find karta hai

STEP 3: TARGETED QUESTIONS (Max 5)
├─ Agent: "Yeh 5 points unclear hain, clarify karo:"
├─
├─ Q1: "Authentication - JWT ya Session?"
├─ Q2: "Rate limiting - Per user ya global?"
├─ Q3: "Pagination - Limit kitna hona chahiye?"
├─ Q4: "Error responses - Standard format?"
├─ Q5: "Caching - Redis ya In-memory?"

STEP 4: USER ANSWERS
├─ User: [Answers provide karta hai]

STEP 5: SPEC UPDATE
├─ Agent: Updated spec.md ko save karta hai
├─ Agent: Ambiguities resolve ho jaate hain
```

### **AGENT'S FINAL REPLY:**

"✅ Specification clarified!

**Updated file:** specs/feedback-system/spec.md
**Changes:** 5 sections clarified

Ready for planning? Run `/sp.plan` next! 🚀"

---

## **AGENT #8: /sp.plan**

### **Purpose:** Implementation plan banao

### **PRACTICAL USE CASE:**

**User Query:** "/sp.plan - Ab plan banao"

### **Step-by-Step Process:**

```
STEP 1: PLAN GENERATION
├─ Agent: Spec.md padhta hai
├─ Agent: Architecture design karta hai

STEP 2: ARCHITECTURAL DECISIONS
├─ Agent identifies:
├─   1. Database choice (Neon Postgres)
├─   2. API design (RESTful)
├─   3. Frontend framework (React)
├─   4. Caching strategy (Redis)
├─   5. Security approach (JWT + CORS)

STEP 3: COMPONENT ARCHITECTURE
├─ Frontend Components
├─ Backend Services
├─ Database Schema
├─ API Endpoints

STEP 4: BUILD PIPELINE
├─ Build steps
├─ Deployment strategy
├─ Testing approach

STEP 5: OUTPUT FILES
├─ specs/feedback-system/plan.md (400+ lines)
├─ specs/feedback-system/data-model.md
├─ specs/feedback-system/research.md
```

### **AGENT'S FINAL REPLY:**

"✅ Implementation plan ready!

**Files:**
- specs/feedback-system/plan.md
- specs/feedback-system/data-model.md
- specs/feedback-system/research.md

**Key Decisions Documented:**
- Neon Postgres for storage
- Redis for caching
- JWT authentication

Next: `/sp.tasks` - break into granular tasks! 📝"

---

## **AGENT #9: /sp.adr <title>**

### **Purpose:** Major architectural decisions document karo

### **PRACTICAL USE CASE:**

**User Query:** "/sp.adr Use Qdrant instead of Pinecone for vector search"

### **Step-by-Step Process:**

```
STEP 1: DECISION IDENTIFICATION
├─ User: "/sp.adr Use Qdrant instead of Pinecone"

STEP 2: ADR DOCUMENT CREATION
├─ Agent creates: history/adr/001-vector-db-choice.md
├─
├─ Content:
├─ ├─ Status: Proposed/Accepted
├─ ├─ Decision: Use Qdrant Cloud
├─ ├─ Rationale:
├─ │  ├─ 1. Free tier with 5GB storage
├─ │  ├─ 2. Easy integration with Python
├─ │  ├─ 3. Hosted service (no DevOps)
├─ │  ├─ 4. Cost: $0 vs Pinecone $25+/month
├─ ├─ Alternatives Considered:
├─ │  ├─ 1. Pinecone (expensive)
├─ │  ├─ 2. Weaviate (complex)
├─ │  ├─ 3. Self-hosted Qdrant (DevOps overhead)
├─ ├─ Consequences:
├─ │  ├─ Pros: Cost-effective, managed service
├─ │  └─ Cons: Rate limits, vendor lock-in
├─ └─ Date: 2024-12-13

STEP 3: VALIDATION
├─ Agent: ADR validate karta hai
├─ Agent: Confirm karta hai ye decision significant hai
```

### **AGENT'S FINAL REPLY:**

"✅ ADR documented!

**File:** history/adr/001-vector-db-choice.md
**Decision:** Use Qdrant Cloud (vs Pinecone)
**Status:** Accepted

**Rationale:** Free tier + easy integration + no DevOps overhead

Reference this when discussing database choices! 📋"

---

## **AGENT #10: /sp.tasks**

### **Purpose:** Break specification into granular tasks

### **PRACTICAL USE CASE:**

**User Query:** "/sp.tasks - Break down into implementation tasks"

### **Step-by-Step Process:**

```
STEP 1: SPECIFICATION ANALYSIS
├─ Agent: spec.md padhta hai
├─ Agent: User stories identify karta hai
├─ Agent: Components breakdown karta hai

STEP 2: TASK DECOMPOSITION
├─ Agent: 75+ tasks generate karta hai
├─ Format: T001 [P] [US1] Description with file path
├─
├─ Example tasks:
├─ ├─ T001 [P] [US1] Create feedback table in Neon
├─ ├─ T002 [P] [US1] Create FeedbackModel in SQLAlchemy
├─ ├─ T003 [US1] Add POST /feedback endpoint
├─ ├─ T004 [P] [US1] Create React FeedbackForm component
├─ ├─ T005 [US1] Integrate form with backend
├─ ├─ T006 [P] [US2] Create analytics dashboard schema
├─ ├─ T007 [US2] Build analytics API endpoint
├─ ├─ T008 [P] [US2] Create analytics frontend component
├─ └─ ... (continuing to T075+)

STEP 3: MVP DEFINITION
├─ Agent: MVP tasks identify karta hai (T001-T020)
├─ These are minimum viable product tasks

STEP 4: PARALLELIZATION
├─ Agent: Tasks ko [P] tag karta hai
├─ [P] = Parallel execution possible
├─ 50+ tasks parallel ho sakte hain

STEP 5: OUTPUT
├─ File: specs/feedback-system/tasks.md
├─ Contains: 75+ tasks, fully organized
```

### **AGENT'S FINAL REPLY:**

"✅ Tasks generated!

**File:** specs/feedback-system/tasks.md
**Total Tasks:** 78
**MVP Tasks:** T001-T020 (13 days)
**Full Implementation:** T001-T078 (30 days)
**Parallel Tasks:** 45 ([P] marked)

Ready to implement? Run `/sp.implement T001-T010` to start! 🚀"

---

## **AGENT #11: /sp.checklist**

### **Purpose:** Validation checklist generate karo

### **PRACTICAL USE CASE:**

**User Query:** "/sp.checklist - Generate validation checklist"

### **Step-by-Step Process:**

```
STEP 1: SPECIFICATION REVIEW
├─ Agent: spec.md padhta hai

STEP 2: CHECKLIST ITEMS GENERATE
├─ Agent creates:
├─ ├─ Functional Requirements Checklist (10 items)
├─ ├─ Non-Functional Checklist (8 items)
├─ ├─ UI/UX Checklist (12 items)
├─ ├─ Performance Checklist (6 items)
├─ ├─ Security Checklist (8 items)
├─ ├─ Testing Checklist (15 items)
├─ └─ Deployment Checklist (10 items)

STEP 3: OUTPUT
├─ File: specs/feedback-system/checklist.md
├─ Format: Markdown checklist items
├─   - [ ] Item 1
├─   - [ ] Item 2
├─   etc.
```

### **AGENT'S FINAL REPLY:**

"✅ Validation checklist ready!

**File:** specs/feedback-system/checklist.md
**Total Items:** 69
**Categories:** 7

Use this checklist during development to verify requirements! ✓"

---

## **AGENT #12: /sp.analyze**

### **Purpose:** Spec → Plan → Tasks alignment check karo

### **PRACTICAL USE CASE:**

**User Query:** "/sp.analyze - Check consistency between all docs"

### **Step-by-Step Process:**

```
STEP 1: CROSS-ARTIFACT ANALYSIS
├─ Agent: spec.md padhta hai
├─ Agent: plan.md padhta hai
├─ Agent: tasks.md padhta hai

STEP 2: CONSISTENCY CHECK
├─ Agent checks:
├─   1. Spec mein jo user stories hain, plan mein ho?
├─   2. Plan mein jo components hain, tasks mein ho?
├─   3. Spec mein success criteria, checklist mein ho?
├─   4. Timeframes realistic hain?

STEP 3: REPORT GENERATION
├─ Output: Consistency report
├─ ├─ ✅ Alignment: 100%
├─ ├─ ✅ Coverage: All requirements covered
├─ ├─ ✅ No orphaned tasks
├─ └─ ⚠️ Warnings (if any)
```

### **AGENT'S FINAL REPLY:**

"✅ Consistency analysis complete!

**Status:** All artifacts aligned
**Spec to Plan:** 100% coverage
**Plan to Tasks:** 100% coverage
**Estimated Effort:** 30 days

Ready to implement! 🚀"

---

## **AGENT #13: /sp.implement**

### **Purpose:** Execute tasks from tasks.md

### **PRACTICAL USE CASE:**

**User Query:** "/sp.implement - Start with T001-T010"

### **Step-by-Step Process:**

```
STEP 1: TASK READING
├─ Agent: tasks.md padhta hai
├─ Agent: T001-T010 identify karta hai

STEP 2: EXECUTION
├─ Agent: Har task ko in order execute karta hai
├─ T001: Create database table
├─   ├─ Read spec.md
├─   ├─ Create SQL migration
├─   ├─ Run migration
├─   └─ Verify in database
├─
├─ T002: Create SQLAlchemy model
├─   ├─ Create models/feedback.py
├─   ├─ Define schema
├─   └─ Test import
├─
├─ ... (continuing for T003-T010)

STEP 3: CODE CHANGES
├─ Agent: Code files create/modify karta hai
├─ Agent: Each change committed hota hai

STEP 4: PROGRESS TRACKING
├─ Agent: tasks.md update karta hai
├─ Format: - [x] T001 (completed) or - [ ] T001 (pending)
```

### **AGENT'S FINAL REPLY:**

"✅ Tasks T001-T010 completed!

**Files created:**
- chatbot-backend/app/models/feedback.py
- chatbot-backend/app/services/feedback_service.py
- chatbot-backend/app/routers/feedback.py
- physical-ai-textbook/src/components/FeedbackForm.tsx

**Database:**
- Neon: 2 new tables created
- Migrations: Applied

**Next:** Run `/sp.implement T011-T020` to continue! 🚀"

---

## **AGENT #14: /sp.constitution**

### **Purpose:** Project principles/constitution create/update karo

### **PRACTICAL USE CASE:**

**User Query:** "/sp.constitution - Update project principles"

### **Step-by-Step Process:**

```
STEP 1: PRINCIPLES DEFINITION
├─ Agent: Current constitution padhta hai
├─ Agent: Updates needed identify karta hai

STEP 2: PRINCIPLES DOCUMENT
├─ Output: .specify/memory/constitution.md
├─
├─ Contains:
├─ ├─ Content Accuracy & Technical Rigor
├─ ├─ Educational Clarity & Accessibility
├─ ├─ Consistency & Standards
├─ ├─ Docusaurus Quality
├─ ├─ Code Example Quality
├─ ├─ UI/UX Excellence
├─ ├─ Deployment & Publishing
├─ └─ AI-Driven Content Standards

STEP 3: VALIDATION
├─ Agent: Constitution validate karta hai
├─ Agent: Ensures it's aligned with project goals
```

### **AGENT'S FINAL REPLY:**

"✅ Constitution updated!

**File:** .specify/memory/constitution.md
**Principles:** 8 core principles defined

All future work will follow these principles! 📜"

---

## **AGENT #15: /sp.phr**

### **Purpose:** Prompt History Record (PHR) create karo

### **PRACTICAL USE CASE:**

**User Query:** "/sp.phr - Document this session"

### **Step-by-Step Process:**

```
STEP 1: PHR METADATA
├─ Agent collects:
├─   ├─ PHR ID (auto-generated)
├─   ├─ Title
├─   ├─ Stage (spec/plan/tasks/implement)
├─   ├─ Date & time
├─   ├─ Model used
├─   ├─ Feature name
├─   ├─ Branch name
├─   └─ Summary

STEP 2: PROMPT CAPTURE
├─ Agent: User prompts capture karta hai
├─ Agent: Model responses capture karta hai

STEP 3: FILE CREATION
├─ Output: history/prompts/feedback-system/001-feedback-spec.spec.prompt.md
├─
├─ Contains:
├─ ├─ Metadata section
├─ ├─ User prompt text (full)
├─ ├─ Model response text (full)
├─ └─ Artifacts list

STEP 4: STORAGE
├─ File stored in: history/prompts/<feature>/<ID>-<slug>.<stage>.prompt.md
```

### **AGENT'S FINAL REPLY:**

"✅ PHR created and documented!

**File:** history/prompts/feedback-system/001-feedback-spec.spec.prompt.md
**ID:** 001
**Stage:** spec
**Feature:** feedback-system

This session is now permanently recorded for future reference! 📚"

---

## **AGENT #16: /sp.git.commit_pr**

### **Purpose:** Autonomous git workflow - commit + PR create karo

### **PRACTICAL USE CASE:**

**User Query:** "/sp.git.commit_pr - Commit and create PR"

### **Step-by-Step Process:**

```
STEP 1: GIT STATUS CHECK
├─ Agent: Modified files check karta hai
├─ Agent: Untracked files check karta hai

STEP 2: COMMIT PREPARATION
├─ Agent: Commit message craft karta hai
├─ Format:
├─ feat: Add feedback system feature
├─
├─ - Create feedback table in Neon
├─ - Add feedback endpoints in FastAPI
├─ - Create React form component
├─ - Integrate with backend
├─ - Add validation & error handling
├─
├─ 🤖 Generated with Claude Code

STEP 3: GIT OPERATIONS
├─ Command 1: git add . (stage all changes)
├─ Command 2: git commit -m "..." (commit)
├─ Command 3: git push origin feedback-system (push)

STEP 4: PR CREATION
├─ Agent: GitHub PR create karta hai
├─ Title: "feat: Add feedback system"
├─ Body:
├─   ## Summary
├─   - Complete feedback collection system
├─   - Analytics dashboard for admins
├─   - Real-time feedback aggregation
├─
├─   ## Changes
├─   - Created feedback tables
├─   - Added 12 API endpoints
├─   - Built React components
├─   - Integrated with backend

STEP 5: OUTPUT
├─ Agent returns: PR URL
├─ Example: https://github.com/username/repo/pull/23
```

### **AGENT'S FINAL REPLY:**

"✅ Changes committed and PR created!

**Commit:** 8 files changed, 245 insertions
**Branch:** feedback-system
**PR URL:** https://github.com/naveed261/GIAIC-HACKATON-1_PROJECT-1/pull/23

**Title:** feat: Add complete feedback system
**Description:** Feedback collection + analytics dashboard

PR ready for review! 🚀"

---

---

# 🎯 COMPLETE AGENT SUMMARY

## **16 AGENTS TOTAL:**

### **GROUP 1: DOMAIN EXPERTS (5)**
1. ✅ Docusaurus Chatbot Agent - UI/Frontend
2. ✅ FastAPI Agent - Backend API
3. ✅ Qdrant Agent - Vector Database
4. ✅ OpenAI Agent - AI/ML Integration
5. ✅ Neon Postgres Agent - Database Management

### **GROUP 2: WORKFLOW (11)**
6. ✅ /sp.specify - Create specification
7. ✅ /sp.clarify - Ask clarification
8. ✅ /sp.plan - Generate plan
9. ✅ /sp.adr - Architecture decisions
10. ✅ /sp.tasks - Break into tasks
11. ✅ /sp.checklist - Validation checklist
12. ✅ /sp.analyze - Consistency check
13. ✅ /sp.implement - Execute implementation
14. ✅ /sp.constitution - Project principles
15. ✅ /sp.phr - Prompt history record
16. ✅ /sp.git.commit_pr - Git workflow

---

## **KAY REMEMBER KARO:**

**Har agent ek specialist hai!**
- Problem aaye → Right agent call karo
- Agent analyze karega
- Solutions dega
- Code examples dega
- Step-by-step guide dega

**Example:** Agar Qdrant search slow ho to `/qdrant-agent` call karo, bilkul detailed solution milaega!

**Bas yeh samajh lo: 16 Experts = 16 Different Specialized Skills! 🤖**



 ACTUAL ORCHESTRATION FLOW:

  USER QUERY
      ↓
  CLAUDE CODE (Main Agent)
      ├─ Query samjhta hai
      ├─ Decide karta hai: Direct solve or Sub-agent?
      │
      ├─ Agar direct: Solve karta hai, reply deta hai
      │
      ├─ Agar complex: Right agent identify karta hai
      │
      └─ Agent ko Task deta hai with context
           ↓
      SUB-AGENT (Specialist)
           ├─ Task receive karta hai
           ├─ Problem analyze karta hai
           ├─ Solution deta hai
           └─ Results Claude Code ko return karta hai
           ↓
      CLAUDE CODE
           ├─ Results process karta hai
           ├─ Format karta hai
           └─ USER KO FINAL ANSWER DETA HAI

  ---
  EXAMPLE:

  USER: "Chatbot widget slow hai"
      ↓
  CLAUDE CODE: "Yeh UI problem hai, /docusaurus-chatbot-agent ko bhejta hoon"
      ↓
  AGENT: "Analyzed! Performance issue: useState caching nahi hai. Fix: useMemo add karo"
      ↓
  CLAUDE CODE: Agent ka solution process karta hai, user ko samjha deta hai
      ↓
  USER: "Yeh raha solution - useMemo add kar, animation optimize kar"

  ---
  KEY POINT:

  Claude Code = Orchestrator (conductor)
  - User query receive
  - Right specialist choose
  - Task assign + supervise
  - Final answer deliver

  Agents = Specialists (musicians)
  - Specific task solve
  - Results return
  - Done