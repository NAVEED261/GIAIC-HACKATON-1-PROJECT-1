"""
SQLAlchemy database models for chat sessions, messages, and users.

Models:
- User: User authentication and profile
- ChatSession: Represents a user chat session
- ChatMessage: Represents individual messages in a conversation
"""

from sqlalchemy import Column, String, Text, Float, Integer, DateTime, ForeignKey, CheckConstraint, TIMESTAMP, Boolean, JSON
from sqlalchemy.orm import relationship, DeclarativeBase
from sqlalchemy.sql import func
import uuid
from datetime import datetime
from typing import List, Optional, Dict, Any


class Base(DeclarativeBase):
    """Base class for all database models."""
    pass


class User(Base):
    """User model for authentication."""

    __tablename__ = "users"

    id = Column(String(36), primary_key=True, default=lambda: str(uuid.uuid4()))
    username = Column(String(255), unique=True, nullable=False, index=True)
    email = Column(String(255), unique=True, nullable=False, index=True)
    password_hash = Column(String(255), nullable=False)
    created_at = Column(
        DateTime(timezone=True),
        server_default=func.now(),
        nullable=False
    )
    updated_at = Column(
        DateTime(timezone=True),
        server_default=func.now(),
        onupdate=func.now(),
        nullable=False
    )
    is_active = Column(Boolean, default=True, nullable=False)

    # Relationship to chat sessions
    chat_sessions = relationship(
        "ChatSession",
        back_populates="user",
        cascade="all, delete-orphan",
        lazy="selectin"
    )

    def __repr__(self) -> str:
        return f"<User(id={self.id}, username={self.username}, email={self.email})>"


class ChatSession(Base):
    """Chat session model for tracking user conversations."""

    __tablename__ = "chat_sessions"

    id = Column(String(36), primary_key=True, default=lambda: str(uuid.uuid4()))
    session_id = Column(String(255), unique=True, nullable=False, index=True)
    user_id = Column(String(36), ForeignKey("users.id", ondelete="CASCADE"), nullable=True, index=True)
    created_at = Column(
        DateTime(timezone=True),
        server_default=func.now(),
        nullable=False
    )
    last_activity = Column(
        DateTime(timezone=True),
        server_default=func.now(),
        onupdate=func.now(),
        nullable=False
    )
    metadata_ = Column("metadata", JSON, default={}, nullable=False)

    # Relationship to user
    user = relationship("User", back_populates="chat_sessions")

    # Relationship to messages
    messages = relationship(
        "ChatMessage",
        back_populates="session",
        cascade="all, delete-orphan",
        lazy="selectin"
    )

    def __repr__(self) -> str:
        return f"<ChatSession(id={self.id}, session_id={self.session_id}, user_id={self.user_id})>"


class ChatMessage(Base):
    """Chat message model for storing conversation history."""

    __tablename__ = "chat_messages"

    id = Column(Integer, primary_key=True, autoincrement=True)
    session_id = Column(
        String(255),
        ForeignKey("chat_sessions.session_id", ondelete="CASCADE"),
        nullable=False,
        index=True
    )
    role = Column(
        String(50),
        CheckConstraint("role IN ('user', 'assistant')"),
        nullable=False
    )
    content = Column(Text, nullable=False)
    sources = Column(JSON, default=[], nullable=False)
    confidence = Column(
        Float,
        CheckConstraint("confidence >= 0.0 AND confidence <= 1.0"),
        nullable=True
    )
    tokens_used = Column(Integer, nullable=True)
    created_at = Column(
        DateTime(timezone=True),
        server_default=func.now(),
        nullable=False
    )

    # Relationship to session
    session = relationship("ChatSession", back_populates="messages")

    def __repr__(self) -> str:
        return f"<ChatMessage(id={self.id}, role={self.role}, session_id={self.session_id})>"

    def to_dict(self) -> Dict[str, Any]:
        """Convert message to dictionary format."""
        return {
            "id": self.id,
            "session_id": self.session_id,
            "role": self.role,
            "content": self.content,
            "sources": self.sources,
            "confidence": self.confidence,
            "tokens_used": self.tokens_used,
            "created_at": self.created_at.isoformat() if self.created_at else None
        }
