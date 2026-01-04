"""
Authentication API endpoints for signup and login.
"""

from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession
import uuid

from app.models.requests import SignupRequest, LoginRequest
from app.models.responses import SignupResponse, LoginResponse, UserResponse, ErrorResponse
from app.services.auth_service import AuthService
from app.db.session import get_db
from app.core.logging import get_logger

logger = get_logger(__name__)

router = APIRouter(prefix="/api/v1", tags=["auth"])


@router.post(
    "/auth/signup",
    response_model=SignupResponse,
    status_code=status.HTTP_201_CREATED,
    responses={
        400: {"model": ErrorResponse, "description": "Invalid input or user already exists"},
        500: {"model": ErrorResponse, "description": "Internal server error"}
    },
    summary="Register a new user",
    description="Create a new user account with username, email, and password"
)
async def signup(
    request: SignupRequest,
    db: AsyncSession = Depends(get_db)
) -> SignupResponse:
    """
    Register a new user account.

    Args:
        request: Signup request with username, email, password
        db: Database session

    Returns:
        SignupResponse with user data

    Raises:
        HTTPException: If signup fails
    """
    try:
        logger.info(f"Signup attempt for username: {request.username}")

        # Register user
        user_data = await AuthService.register_user(
            db=db,
            username=request.username,
            email=request.email,
            password=request.password
        )

        logger.info(f"User registered successfully: {request.username}")

        return SignupResponse(
            success=True,
            message="User registered successfully",
            user=UserResponse(
                id=user_data["id"],
                username=user_data["username"],
                email=user_data["email"],
                created_at=user_data["created_at"]
            )
        )

    except ValueError as e:
        logger.warning(f"Signup validation error: {e}")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail={
                "error": "ValidationError",
                "message": str(e)
            }
        )

    except Exception as e:
        logger.error(f"Signup error: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "error": "InternalServerError",
                "message": "Failed to register user"
            }
        )


@router.post(
    "/auth/login",
    response_model=LoginResponse,
    status_code=status.HTTP_200_OK,
    responses={
        401: {"model": ErrorResponse, "description": "Invalid credentials"},
        500: {"model": ErrorResponse, "description": "Internal server error"}
    },
    summary="Login user",
    description="Authenticate user with username/email and password"
)
async def login(
    request: LoginRequest,
    db: AsyncSession = Depends(get_db)
) -> LoginResponse:
    """
    Authenticate a user and return login response.

    Args:
        request: Login request with username and password
        db: Database session

    Returns:
        LoginResponse with user data and session ID

    Raises:
        HTTPException: If login fails
    """
    try:
        logger.info(f"Login attempt for username: {request.username}")

        # Authenticate user
        user_data = await AuthService.login_user(
            db=db,
            username=request.username,
            password=request.password
        )

        if not user_data:
            logger.warning(f"Login failed for username: {request.username}")
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail={
                    "error": "AuthenticationError",
                    "message": "Invalid username or password"
                }
            )

        # Generate session ID
        session_id = f"session-{request.username}-{str(uuid.uuid4())[:8]}"

        logger.info(f"User logged in successfully: {request.username}")

        return LoginResponse(
            success=True,
            message="Login successful",
            user=UserResponse(
                id=user_data["id"],
                username=user_data["username"],
                email=user_data["email"],
                created_at=user_data["created_at"]
            ),
            session_id=session_id
        )

    except HTTPException:
        raise

    except Exception as e:
        logger.error(f"Login error: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "error": "InternalServerError",
                "message": "Failed to login user"
            }
        )
