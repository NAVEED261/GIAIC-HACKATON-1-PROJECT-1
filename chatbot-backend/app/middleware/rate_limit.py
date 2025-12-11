"""
Rate limiting middleware for API endpoints.

Implements per-session rate limiting using in-memory storage.
"""

from fastapi import Request, HTTPException, status
from fastapi.responses import JSONResponse
from starlette.middleware.base import BaseHTTPMiddleware
from typing import Dict
from datetime import datetime, timedelta
from collections import defaultdict
import asyncio

from app.core.config import settings
from app.core.logging import get_logger

logger = get_logger(__name__)


class RateLimitMiddleware(BaseHTTPMiddleware):
    """
    Rate limiting middleware using sliding window algorithm.

    Limits requests per session_id based on configuration.
    """

    def __init__(self, app, requests_limit: int = None, window_seconds: int = None):
        """
        Initialize rate limiter.

        Args:
            app: FastAPI application
            requests_limit: Maximum requests per window (default from settings)
            window_seconds: Time window in seconds (default from settings)
        """
        super().__init__(app)

        self.requests_limit = requests_limit or settings.RATE_LIMIT_REQUESTS
        self.window_seconds = window_seconds or settings.RATE_LIMIT_WINDOW

        # In-memory storage: {session_id: [timestamp1, timestamp2, ...]}
        self.request_log: Dict[str, list] = defaultdict(list)

        # Start cleanup task
        asyncio.create_task(self._cleanup_loop())

        logger.info(
            f"Rate limiter initialized: {self.requests_limit} requests "
            f"per {self.window_seconds} seconds"
        )

    async def dispatch(self, request: Request, call_next):
        """
        Process request with rate limiting.

        Args:
            request: Incoming request
            call_next: Next middleware/handler

        Returns:
            Response or 429 Too Many Requests error
        """
        # Only apply rate limiting to /chat endpoint
        if not request.url.path.startswith("/api/v1/chat"):
            return await call_next(request)

        # Extract session_id from request body
        session_id = await self._extract_session_id(request)

        if not session_id:
            # If no session_id, allow request but log warning
            logger.warning("No session_id found in request, skipping rate limit")
            return await call_next(request)

        # Check rate limit
        if self._is_rate_limited(session_id):
            logger.warning(f"Rate limit exceeded for session: {session_id}")

            return JSONResponse(
                status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                content={
                    "error": "RateLimitExceeded",
                    "message": (
                        f"Too many requests. Limit: {self.requests_limit} "
                        f"requests per {self.window_seconds // 3600} hour(s). "
                        "Please try again later."
                    ),
                    "details": {
                        "limit": self.requests_limit,
                        "window_seconds": self.window_seconds,
                        "session_id": session_id
                    }
                }
            )

        # Record request
        self._record_request(session_id)

        # Process request
        response = await call_next(request)

        return response

    async def _extract_session_id(self, request: Request) -> str:
        """
        Extract session_id from request body.

        Args:
            request: FastAPI request

        Returns:
            Session ID or None
        """
        try:
            # Read body
            body = await request.body()

            # Parse JSON to get session_id
            if body:
                import json
                data = json.loads(body)
                session_id = data.get("session_id")

                # Important: Re-attach body for downstream handlers
                async def receive():
                    return {"type": "http.request", "body": body}

                request._receive = receive

                return session_id

        except Exception as e:
            logger.error(f"Failed to extract session_id: {e}")

        return None

    def _is_rate_limited(self, session_id: str) -> bool:
        """
        Check if session has exceeded rate limit.

        Args:
            session_id: Session identifier

        Returns:
            True if rate limited, False otherwise
        """
        now = datetime.utcnow()
        window_start = now - timedelta(seconds=self.window_seconds)

        # Get request timestamps for this session
        timestamps = self.request_log[session_id]

        # Remove old timestamps outside window
        timestamps[:] = [ts for ts in timestamps if ts > window_start]

        # Check if limit exceeded
        return len(timestamps) >= self.requests_limit

    def _record_request(self, session_id: str) -> None:
        """
        Record a request for rate limiting.

        Args:
            session_id: Session identifier
        """
        self.request_log[session_id].append(datetime.utcnow())

    async def _cleanup_loop(self) -> None:
        """
        Periodic cleanup of old request logs.

        Runs every hour to remove expired entries and free memory.
        """
        while True:
            await asyncio.sleep(3600)  # Run every hour

            try:
                now = datetime.utcnow()
                window_start = now - timedelta(seconds=self.window_seconds)

                # Clean up old entries
                sessions_to_delete = []

                for session_id, timestamps in self.request_log.items():
                    # Remove old timestamps
                    timestamps[:] = [ts for ts in timestamps if ts > window_start]

                    # Mark empty sessions for deletion
                    if not timestamps:
                        sessions_to_delete.append(session_id)

                # Delete empty sessions
                for session_id in sessions_to_delete:
                    del self.request_log[session_id]

                if sessions_to_delete:
                    logger.info(f"Cleaned up {len(sessions_to_delete)} expired rate limit entries")

            except Exception as e:
                logger.error(f"Rate limit cleanup error: {e}")
