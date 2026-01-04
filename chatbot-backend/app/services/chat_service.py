"""
OpenAI Chat Service for generating AI responses.

Provides async methods to generate chat completions using OpenAI's gpt-4o-mini model.
"""

from typing import List, Dict, Any, Optional
from openai import AsyncOpenAI
from ..core.config import settings
from ..core.logging import get_logger

logger = get_logger(__name__)


class ChatService:
    """Service for generating chat responses using OpenAI API."""

    def __init__(self, api_key: str = settings.OPENAI_API_KEY):
        """
        Initialize chat service with OpenAI API key.

        Args:
            api_key: OpenAI API key (defaults to settings.OPENAI_API_KEY)
        """
        self.client = AsyncOpenAI(api_key=api_key)
        self.model = settings.OPENAI_CHAT_MODEL
        self.temperature = settings.OPENAI_TEMPERATURE
        self.max_tokens = settings.OPENAI_MAX_TOKENS
        logger.info(f"Initialized ChatService with model: {self.model}")

    async def generate_answer(
        self,
        query: str,
        context_chunks: List[Dict[str, Any]],
        chat_history: Optional[List[Dict[str, str]]] = None
    ) -> Dict[str, Any]:
        """
        Generate an answer to a query using retrieved context.

        Args:
            query: User's question
            context_chunks: List of retrieved document chunks with metadata
            chat_history: Optional previous messages for context

        Returns:
            Dictionary containing:
                - answer: Generated response text
                - tokens_used: Total tokens consumed
                - model: Model used for generation
        """
        try:
            # Build context from chunks
            context_text = self._build_context(context_chunks)

            # Create system prompt
            system_prompt = self._create_system_prompt(context_text)

            # Build message history
            messages = [{"role": "system", "content": system_prompt}]

            # Add chat history if provided
            if chat_history:
                messages.extend(chat_history)

            # Add current user query
            messages.append({"role": "user", "content": query})

            # Generate completion
            logger.info(f"Generating answer for query: '{query[:100]}...'")

            response = await self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                temperature=self.temperature,
                max_tokens=self.max_tokens
            )

            answer = response.choices[0].message.content
            tokens_used = response.usage.total_tokens

            logger.info(
                f"Generated answer (tokens: {tokens_used}, "
                f"context chunks: {len(context_chunks)})"
            )

            return {
                "answer": answer,
                "tokens_used": tokens_used,
                "model": self.model
            }

        except Exception as e:
            logger.error(f"Failed to generate answer: {e}")
            raise

    def _build_context(self, chunks: List[Dict[str, Any]]) -> str:
        """
        Build context text from retrieved chunks.

        Args:
            chunks: List of chunks with 'payload' containing text and metadata

        Returns:
            Formatted context string
        """
        if not chunks:
            return "No relevant context found in the textbook."

        context_parts = []

        for i, chunk in enumerate(chunks, 1):
            payload = chunk.get("payload", {})
            text = payload.get("text", "")
            chapter = payload.get("chapter", "Unknown")
            week = payload.get("week", "?")
            module = payload.get("module", "?")

            context_parts.append(
                f"[Source {i} - Week {week}, Module {module}: {chapter}]\n{text}\n"
            )

        return "\n".join(context_parts)

    def _create_system_prompt(self, context: str) -> str:
        """
        Create system prompt with context and instructions (OPTIMIZED FOR SPEED).

        Args:
            context: Retrieved context text

        Returns:
            System prompt string
        """
        return f"""You are a Physical AI teaching assistant. Answer questions directly using the textbook content below.

Context:
{context}

Be concise and clear."""

    async def close(self) -> None:
        """Close the OpenAI client connection."""
        await self.client.close()
        logger.info("ChatService client closed")
