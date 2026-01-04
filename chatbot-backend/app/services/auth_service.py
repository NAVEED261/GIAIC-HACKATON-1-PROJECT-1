"""
Authentication service for user registration and login.
"""

import bcrypt
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select
from ..db.models import User
from ..core.logging import get_logger
from typing import Optional, Dict, Any

logger = get_logger(__name__)


class AuthService:
    """Service for user authentication and password management."""

    @staticmethod
    def hash_password(password: str) -> str:
        """Hash a password using bcrypt."""
        # Encode password to bytes and hash
        password_bytes = password.encode('utf-8')
        salt = bcrypt.gensalt()
        hashed = bcrypt.hashpw(password_bytes, salt)
        return hashed.decode('utf-8')

    @staticmethod
    def verify_password(plain_password: str, hashed_password: str) -> bool:
        """Verify a password against its hash."""
        password_bytes = plain_password.encode('utf-8')
        hashed_bytes = hashed_password.encode('utf-8')
        return bcrypt.checkpw(password_bytes, hashed_bytes)

    @staticmethod
    async def register_user(
        db: AsyncSession,
        username: str,
        email: str,
        password: str
    ) -> Dict[str, Any]:
        """
        Register a new user.

        Args:
            db: Database session
            username: Username
            email: Email address
            password: Plain password (will be hashed)

        Returns:
            Dictionary with user info

        Raises:
            ValueError: If user already exists
        """
        # Check if user exists
        result = await db.execute(
            select(User).where(
                (User.username == username) | (User.email == email)
            )
        )
        existing_user = result.scalar_one_or_none()

        if existing_user:
            raise ValueError("Username or email already exists")

        # Create new user
        user = User(
            username=username,
            email=email,
            password_hash=AuthService.hash_password(password)
        )

        db.add(user)
        await db.commit()
        await db.refresh(user)

        logger.info(f"User registered: {username}")

        return {
            "id": user.id,
            "username": user.username,
            "email": user.email,
            "created_at": user.created_at.isoformat()
        }

    @staticmethod
    async def login_user(
        db: AsyncSession,
        username: str,
        password: str
    ) -> Optional[Dict[str, Any]]:
        """
        Authenticate a user.

        Args:
            db: Database session
            username: Username
            password: Plain password

        Returns:
            Dictionary with user info if successful, None otherwise
        """
        # Find user
        result = await db.execute(
            select(User).where(User.username == username)
        )
        user = result.scalar_one_or_none()

        if not user:
            logger.warning(f"Login attempt for non-existent user: {username}")
            return None

        # Verify password
        if not AuthService.verify_password(password, user.password_hash):
            logger.warning(f"Invalid password for user: {username}")
            return None

        if not user.is_active:
            logger.warning(f"Login attempt for inactive user: {username}")
            return None

        logger.info(f"User logged in: {username}")

        return {
            "id": user.id,
            "username": user.username,
            "email": user.email,
            "created_at": user.created_at.isoformat()
        }

    @staticmethod
    async def get_user_by_id(
        db: AsyncSession,
        user_id: str
    ) -> Optional[Dict[str, Any]]:
        """Get user by ID."""
        result = await db.execute(
            select(User).where(User.id == user_id)
        )
        user = result.scalar_one_or_none()

        if not user:
            return None

        return {
            "id": user.id,
            "username": user.username,
            "email": user.email,
            "created_at": user.created_at.isoformat()
        }
