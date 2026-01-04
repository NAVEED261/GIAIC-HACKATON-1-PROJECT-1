"""
Pydantic models for API responses.

Defines response schemas for chat, history, health, and error endpoints.
"""

from pydantic import BaseModel, Field
from typing import List, Optional, Any, Dict
from datetime import datetime


class Source(BaseModel):
    """Source citation from textbook."""

    chapter: str = Field(
        ...,
        description="Chapter title from the textbook",
        examples=["ROS 2 Fundamentals"]
    )

    module: Optional[int] = Field(
        None,
        ge=1,
        le=4,
        description="Module number (1-4)",
        examples=[1]
    )

    week: Optional[int] = Field(
        None,
        ge=1,
        le=13,
        description="Week number (1-13)",
        examples=[3]
    )

    score: float = Field(
        ...,
        ge=0.0,
        le=1.0,
        description="Relevance score (0.0-1.0)",
        examples=[0.92]
    )

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "chapter": "ROS 2 Fundamentals",
                    "module": 1,
                    "week": 3,
                    "score": 0.92
                }
            ]
        }
    }


class ChatResponse(BaseModel):
    """Response model for POST /api/v1/chat endpoint."""

    answer: str = Field(
        ...,
        description="AI-generated answer to the user's question",
        examples=["ROS 2 is the next generation Robot Operating System..."]
    )

    sources: List[Source] = Field(
        ...,
        description="List of source citations from the textbook",
        examples=[[{
            "chapter": "ROS 2 Fundamentals",
            "module": 1,
            "week": 3,
            "score": 0.92
        }]]
    )

    session_id: str = Field(
        ...,
        description="Session identifier for tracking conversation",
        examples=["session-1234567890-abc"]
    )

    confidence: float = Field(
        ...,
        ge=0.0,
        le=1.0,
        description="Confidence score based on source relevance (0.0-1.0)",
        examples=[0.92]
    )

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "answer": "ROS 2 is an open-source framework for robot development...",
                    "sources": [
                        {
                            "chapter": "ROS 2 Fundamentals",
                            "module": 1,
                            "week": 3,
                            "score": 0.92
                        },
                        {
                            "chapter": "Architecture Overview",
                            "module": 1,
                            "week": 2,
                            "score": 0.85
                        }
                    ],
                    "session_id": "session-1234567890-abc",
                    "confidence": 0.92
                }
            ]
        }
    }


class MessageResponse(BaseModel):
    """Individual message in chat history."""

    id: int = Field(..., description="Message ID")
    role: str = Field(..., description="Message role (user or assistant)")
    content: str = Field(..., description="Message content")
    sources: List[Source] = Field(default=[], description="Sources (for assistant messages)")
    confidence: Optional[float] = Field(None, description="Confidence score (for assistant messages)")
    tokens_used: Optional[int] = Field(None, description="Tokens consumed (for assistant messages)")
    created_at: str = Field(..., description="Message timestamp (ISO 8601)")


class HistoryResponse(BaseModel):
    """Response model for GET /api/v1/history/{session_id} endpoint."""

    session_id: str = Field(
        ...,
        description="Session identifier",
        examples=["session-1234567890-abc"]
    )

    messages: List[MessageResponse] = Field(
        ...,
        description="List of messages in chronological order"
    )

    total_messages: int = Field(
        ...,
        description="Total number of messages in the session",
        examples=[10]
    )

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "session_id": "session-1234567890-abc",
                    "messages": [
                        {
                            "id": 1,
                            "role": "user",
                            "content": "What is ROS 2?",
                            "sources": [],
                            "confidence": None,
                            "tokens_used": None,
                            "created_at": "2025-12-11T10:00:00Z"
                        },
                        {
                            "id": 2,
                            "role": "assistant",
                            "content": "ROS 2 is...",
                            "sources": [{"chapter": "ROS 2 Fundamentals", "module": 1, "week": 3, "score": 0.92}],
                            "confidence": 0.92,
                            "tokens_used": 150,
                            "created_at": "2025-12-11T10:00:05Z"
                        }
                    ],
                    "total_messages": 2
                }
            ]
        }
    }


class HealthResponse(BaseModel):
    """Response model for GET /api/v1/health endpoint."""

    status: str = Field(
        ...,
        description="Health status",
        examples=["healthy"]
    )

    service: str = Field(
        ...,
        description="Service name",
        examples=["chatbot-backend"]
    )

    version: str = Field(
        ...,
        description="API version",
        examples=["1.0.0"]
    )

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "status": "healthy",
                    "service": "chatbot-backend",
                    "version": "1.0.0"
                }
            ]
        }
    }


class UserResponse(BaseModel):
    """Response model for user data."""

    id: str = Field(..., description="User ID")
    username: str = Field(..., description="Username")
    email: str = Field(..., description="Email address")
    created_at: str = Field(..., description="Account creation timestamp")


class SignupResponse(BaseModel):
    """Response model for POST /api/v1/auth/signup endpoint."""

    success: bool = Field(..., description="Whether signup was successful")
    message: str = Field(..., description="Status message")
    user: UserResponse = Field(..., description="User data")

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "success": True,
                    "message": "User registered successfully",
                    "user": {
                        "id": "550e8400-e29b-41d4-a716-446655440000",
                        "username": "john_doe",
                        "email": "john@example.com",
                        "created_at": "2025-12-17T10:00:00"
                    }
                }
            ]
        }
    }


class LoginResponse(BaseModel):
    """Response model for POST /api/v1/auth/login endpoint."""

    success: bool = Field(..., description="Whether login was successful")
    message: str = Field(..., description="Status message")
    user: UserResponse = Field(..., description="User data")
    session_id: str = Field(..., description="Session ID for chatbot")

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "success": True,
                    "message": "Login successful",
                    "user": {
                        "id": "550e8400-e29b-41d4-a716-446655440000",
                        "username": "john_doe",
                        "email": "john@example.com",
                        "created_at": "2025-12-17T10:00:00"
                    },
                    "session_id": "session-john-doe-12345"
                }
            ]
        }
    }


class ErrorResponse(BaseModel):
    """Response model for error responses."""

    error: str = Field(
        ...,
        description="Error type",
        examples=["ValidationError"]
    )

    message: str = Field(
        ...,
        description="Error message",
        examples=["Query cannot be empty"]
    )

    details: Optional[Dict[str, Any]] = Field(
        None,
        description="Additional error details"
    )

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "error": "ValidationError",
                    "message": "Query cannot be empty",
                    "details": {"field": "query", "value": ""}
                }
            ]
        }
    }
