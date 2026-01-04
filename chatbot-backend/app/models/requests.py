"""
Pydantic models for API request validation.

Defines request schemas for chat and history endpoints.
"""

from pydantic import BaseModel, Field, field_validator
from typing import Optional


class ChatRequest(BaseModel):
    """Request model for POST /api/v1/chat endpoint."""

    query: str = Field(
        ...,
        min_length=1,
        max_length=2000,
        description="User's question about the course content",
        examples=["What is ROS 2 and how does it differ from ROS 1?"]
    )

    session_id: str = Field(
        ...,
        min_length=1,
        max_length=255,
        description="Unique session identifier for tracking conversation history",
        examples=["session-1234567890-abc"]
    )

    @field_validator("query")
    @classmethod
    def query_not_empty(cls, v: str) -> str:
        """Validate that query is not just whitespace."""
        if not v.strip():
            raise ValueError("Query cannot be empty or whitespace only")
        return v.strip()

    @field_validator("session_id")
    @classmethod
    def session_id_format(cls, v: str) -> str:
        """Validate session_id format."""
        if not v.strip():
            raise ValueError("Session ID cannot be empty")
        return v.strip()

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "query": "What is ROS 2 and how does it differ from ROS 1?",
                    "session_id": "session-1234567890-abc"
                },
                {
                    "query": "Explain the concept of digital twins in robotics",
                    "session_id": "session-9876543210-xyz"
                }
            ]
        }
    }


class HistoryRequest(BaseModel):
    """Request model for GET /api/v1/history/{session_id} endpoint."""

    session_id: str = Field(
        ...,
        description="Session identifier to retrieve history for"
    )

    limit: Optional[int] = Field(
        default=50,
        ge=1,
        le=100,
        description="Maximum number of messages to retrieve"
    )

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "session_id": "session-1234567890-abc",
                    "limit": 20
                }
            ]
        }
    }


class SignupRequest(BaseModel):
    """Request model for POST /api/v1/auth/signup endpoint."""

    username: str = Field(
        ...,
        min_length=3,
        max_length=50,
        description="Username (3-50 characters)",
        examples=["john_doe"]
    )

    email: str = Field(
        ...,
        description="Email address",
        examples=["john@example.com"]
    )

    password: str = Field(
        ...,
        min_length=6,
        max_length=100,
        description="Password (minimum 6 characters)",
        examples=["SecurePassword123"]
    )

    @field_validator("username")
    @classmethod
    def username_valid(cls, v: str) -> str:
        """Validate username."""
        v = v.strip()
        if not v.isalnum() and "_" not in v:
            raise ValueError("Username must contain only alphanumeric characters and underscores")
        return v

    @field_validator("email")
    @classmethod
    def email_valid(cls, v: str) -> str:
        """Validate email format."""
        if "@" not in v or "." not in v:
            raise ValueError("Invalid email address")
        return v.strip()

    @field_validator("password")
    @classmethod
    def password_valid(cls, v: str) -> str:
        """Validate password strength."""
        if len(v) < 6:
            raise ValueError("Password must be at least 6 characters")
        return v


class LoginRequest(BaseModel):
    """Request model for POST /api/v1/auth/login endpoint."""

    username: str = Field(
        ...,
        description="Username or email",
        examples=["john_doe"]
    )

    password: str = Field(
        ...,
        description="Password",
        examples=["SecurePassword123"]
    )

    @field_validator("username", "password")
    @classmethod
    def fields_not_empty(cls, v: str) -> str:
        """Validate fields are not empty."""
        if not v.strip():
            raise ValueError("Field cannot be empty")
        return v.strip()
