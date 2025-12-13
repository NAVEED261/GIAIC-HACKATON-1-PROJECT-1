# OpenAI Agent Skill

**Purpose:** Expert in OpenAI API for embeddings generation and chat completions with RAG context injection.

## Expertise

- OpenAI Python SDK (`openai>=1.0.0`)
- `text-embedding-3-small` model for embeddings (1536 dimensions)
- `gpt-4o-mini` for chat completions (cost-effective, fast)
- System prompt engineering for educational chatbot
- Context injection with RAG chunks
- Token counting and cost optimization
- Rate limiting and error handling
- Async operations with `asyncio`

## OpenAI API Setup

```bash
# Install SDK
pip install openai>=1.0.0

# Set API key
export OPENAI_API_KEY="sk-..."
```

## Code Patterns

### 1. Embedding Service

```python
from openai import AsyncOpenAI
from typing import List
import logging

class EmbeddingService:
    def __init__(self, api_key: str):
        self.client = AsyncOpenAI(api_key=api_key)
        self.model = "text-embedding-3-small"
        self.logger = logging.getLogger(__name__)

    async def embed_text(self, text: str) -> List[float]:
        """
        Generate embedding for single text.

        Args:
            text: Input text to embed

        Returns:
            Embedding vector (1536 dimensions)
        """
        try:
            response = await self.client.embeddings.create(
                model=self.model,
                input=text
            )
            return response.data[0].embedding
        except Exception as e:
            self.logger.error(f"Embedding generation failed: {e}")
            raise

    async def embed_batch(self, texts: List[str], batch_size: int = 100) -> List[List[float]]:
        """
        Generate embeddings for batch of texts.

        Args:
            texts: List of texts to embed
            batch_size: Max texts per API call (OpenAI limit: 2048)

        Returns:
            List of embedding vectors
        """
        all_embeddings = []

        for i in range(0, len(texts), batch_size):
            batch = texts[i:i + batch_size]
            try:
                response = await self.client.embeddings.create(
                    model=self.model,
                    input=batch
                )
                embeddings = [item.embedding for item in response.data]
                all_embeddings.extend(embeddings)
                self.logger.info(f"Embedded batch {i//batch_size + 1}, {len(batch)} texts")
            except Exception as e:
                self.logger.error(f"Batch embedding failed: {e}")
                raise

        return all_embeddings
```

### 2. Chat Completion Service

```python
class ChatService:
    def __init__(self, api_key: str):
        self.client = AsyncOpenAI(api_key=api_key)
        self.model = "gpt-4o-mini"  # Cost-effective
        self.logger = logging.getLogger(__name__)

    async def generate_answer(
        self,
        query: str,
        context_chunks: List[Dict],
        temperature: float = 0.3,
        max_tokens: int = 500
    ) -> Dict:
        """
        Generate answer using RAG context.

        Args:
            query: User's question
            context_chunks: Retrieved chunks from Qdrant
            temperature: Creativity (0.0-2.0, default 0.3 for accuracy)
            max_tokens: Max response length

        Returns:
            Dict with answer, sources, confidence
        """
        # Build context string
        context_str = self._format_context(context_chunks)

        # System prompt
        system_prompt = """You are a helpful teaching assistant for the Physical AI Textbook course.
Answer questions based ONLY on the provided context from the textbook.
If the context doesn't contain the answer, say "I don't have information on this topic in the textbook."
Cite the chapter/module for each answer.
Provide code examples when relevant.
Keep answers concise (2-3 paragraphs max)."""

        # User message with context
        user_message = f"""Context from textbook:
{context_str}

Question: {query}

Answer the question based on the context above. Cite sources."""

        try:
            response = await self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_message}
                ],
                temperature=temperature,
                max_tokens=max_tokens
            )

            answer = response.choices[0].message.content
            confidence = self._calculate_confidence(context_chunks)

            return {
                "answer": answer,
                "sources": [
                    {"chapter": chunk["chapter"], "score": chunk["score"]}
                    for chunk in context_chunks
                ],
                "confidence": confidence,
                "usage": {
                    "prompt_tokens": response.usage.prompt_tokens,
                    "completion_tokens": response.usage.completion_tokens,
                    "total_tokens": response.usage.total_tokens
                }
            }
        except Exception as e:
            self.logger.error(f"Chat completion failed: {e}")
            raise

    def _format_context(self, chunks: List[Dict]) -> str:
        """Format context chunks for LLM."""
        formatted = []
        for i, chunk in enumerate(chunks, 1):
            formatted.append(
                f"[Source {i}] Chapter: {chunk['chapter']} (Module {chunk['module']}, Week {chunk['week']})\n"
                f"{chunk['text']}\n"
            )
        return "\n".join(formatted)

    def _calculate_confidence(self, chunks: List[Dict]) -> float:
        """Calculate confidence based on similarity scores."""
        if not chunks:
            return 0.0
        avg_score = sum(c["score"] for c in chunks) / len(chunks)
        return round(avg_score, 2)
```

### 3. RAG Orchestration Service

```python
class RAGService:
    def __init__(
        self,
        embedding_service: EmbeddingService,
        vector_service: VectorSearchService,
        chat_service: ChatService
    ):
        self.embedding_service = embedding_service
        self.vector_service = vector_service
        self.chat_service = chat_service
        self.logger = logging.getLogger(__name__)

    async def generate_answer(
        self,
        query: str,
        session_id: str,
        context: Optional[str] = None
    ) -> Dict:
        """
        Full RAG pipeline: embed query → search → generate answer.

        Args:
            query: User's question
            session_id: Session ID for chat history
            context: Optional selected text from frontend

        Returns:
            ChatResponse dict
        """
        # Step 1: Generate query embedding
        query_embedding = await self.embedding_service.embed_text(query)

        # Step 2: Search similar chunks
        chunks = await self.vector_service.search(
            query_vector=query_embedding,
            top_k=5,
            score_threshold=0.7
        )

        # Step 3: Check if context is relevant
        if not chunks or chunks[0]["score"] < 0.7:
            return {
                "answer": "I don't have information on this topic in the Physical AI Textbook. Could you rephrase your question?",
                "sources": [],
                "session_id": session_id,
                "confidence": 0.0
            }

        # Step 4: Generate answer with LLM
        result = await self.chat_service.generate_answer(
            query=query,
            context_chunks=chunks
        )

        # Step 5: Add session ID
        result["session_id"] = session_id

        # Log for monitoring
        self.logger.info(
            f"Query: {query[:50]}... | "
            f"Retrieved {len(chunks)} chunks | "
            f"Confidence: {result['confidence']} | "
            f"Tokens: {result['usage']['total_tokens']}"
        )

        return result
```

## System Prompt Templates

### Educational Chatbot Prompt

```python
SYSTEM_PROMPT = """You are a helpful teaching assistant for the Physical AI Textbook course covering ROS 2, Digital Twin Simulation, NVIDIA Isaac Sim, and Vision-Language-Action models for humanoid robotics.

Your role:
- Answer questions based ONLY on the provided context from the textbook
- If the context doesn't contain the answer, say "I don't have information on this topic in the textbook"
- Cite the chapter and module for each answer
- Provide code examples when relevant (use Python for ROS 2, bash for commands)
- Keep answers concise (2-3 paragraphs max)
- Use encouraging, supportive tone
- Suggest related chapters if user wants to learn more

Guidelines:
- Never make up information not in the context
- If asked about topics outside the textbook scope, politely decline
- For setup questions, refer to specific hardware/software guides
- For debugging, ask clarifying questions about error messages"""
```

## Token Counting and Cost Optimization

```python
import tiktoken

def count_tokens(text: str, model: str = "gpt-4o-mini") -> int:
    """Count tokens in text for cost estimation."""
    encoding = tiktoken.encoding_for_model(model)
    return len(encoding.encode(text))

def estimate_cost(prompt_tokens: int, completion_tokens: int) -> float:
    """
    Estimate API cost.

    gpt-4o-mini pricing (as of Dec 2024):
    - Input: $0.15 per 1M tokens
    - Output: $0.60 per 1M tokens
    """
    input_cost = (prompt_tokens / 1_000_000) * 0.15
    output_cost = (completion_tokens / 1_000_000) * 0.60
    return input_cost + output_cost
```

## Error Handling

```python
from openai import RateLimitError, APIError, AuthenticationError
import asyncio

async def generate_with_retry(
    client: AsyncOpenAI,
    max_retries: int = 3,
    **kwargs
) -> Any:
    """Retry logic for OpenAI API calls."""
    for attempt in range(max_retries):
        try:
            return await client.chat.completions.create(**kwargs)
        except RateLimitError:
            wait_time = 2 ** attempt  # Exponential backoff
            logging.warning(f"Rate limit hit, retrying in {wait_time}s...")
            await asyncio.sleep(wait_time)
        except AuthenticationError:
            logging.error("Invalid API key")
            raise
        except APIError as e:
            logging.error(f"API error: {e}")
            raise

    raise Exception("Max retries exceeded")
```

## Best Practices

1. **Use `gpt-4o-mini`** for cost efficiency (20x cheaper than GPT-4)
2. **Set temperature = 0.3** for factual answers (balance creativity vs accuracy)
3. **Limit max_tokens = 500** to control costs and enforce concise answers
4. **Always inject context** from vector search (don't rely on model's training data)
5. **Monitor token usage** to track costs
6. **Implement retries** for rate limits and transient errors
7. **Cache embeddings** for frequently asked questions (future optimization)

## Testing

```python
import pytest
from app.services.embeddings import EmbeddingService

@pytest.mark.asyncio
async def test_embedding_generation():
    service = EmbeddingService(api_key=os.getenv("OPENAI_API_KEY"))

    text = "What is ROS 2?"
    embedding = await service.embed_text(text)

    assert len(embedding) == 1536  # text-embedding-3-small dimension
    assert all(isinstance(x, float) for x in embedding)

@pytest.mark.asyncio
async def test_chat_completion():
    chat_service = ChatService(api_key=os.getenv("OPENAI_API_KEY"))

    chunks = [{
        "text": "ROS 2 is an open-source framework...",
        "chapter": "ROS 2 Fundamentals",
        "module": 1,
        "week": 3,
        "score": 0.85
    }]

    result = await chat_service.generate_answer(
        query="What is ROS 2?",
        context_chunks=chunks
    )

    assert "answer" in result
    assert result["confidence"] > 0.0
```

## When to Use This Agent

- Generating embeddings for documents or queries
- Implementing RAG chat completions
- Optimizing prompts for educational context
- Handling OpenAI API errors and rate limits
- Monitoring token usage and costs
- Testing LLM responses for accuracy
