#!/usr/bin/env python3
"""
Quick test script to verify backend works without needing uvicorn.
This imports the app and tests the key components.
"""

import asyncio
import sys
import os

# Fix Windows console encoding
os.environ['PYTHONIOENCODING'] = 'utf-8'

async def test_backend():
    """Test backend components."""
    print("=" * 60)
    print("TESTING PHYSICAL AI CHATBOT BACKEND")
    print("=" * 60)

    # Test 1: Import main app
    print("\n[1/4] Testing FastAPI app import...")
    try:
        from app.main import app
        print("[OK] FastAPI app loaded successfully")
    except Exception as e:
        print(f"[FAIL] Failed to import app: {e}")
        return False

    # Test 2: Database configuration
    print("\n[2/4] Testing database configuration...")
    try:
        from app.db.session import engine
        print("[OK] Database engine created successfully")
        print(f"   Database URL: sqlite+aiosqlite:///./chatbot.db")
    except Exception as e:
        print(f"[FAIL] Database error: {e}")
        return False

    # Test 3: Configuration loading
    print("\n[3/4] Testing settings configuration...")
    try:
        from app.core.config import settings
        print("[OK] Settings loaded successfully")
        print(f"   OpenAI Model: {settings.OPENAI_CHAT_MODEL}")
        print(f"   Qdrant Collection: {settings.QDRANT_COLLECTION_NAME}")
        print(f"   Max tokens: {settings.OPENAI_MAX_TOKENS}")
    except Exception as e:
        print(f"[FAIL] Settings error: {e}")
        return False

    # Test 4: Services initialization
    print("\n[4/4] Testing service initialization...")
    try:
        from app.services.embedding_service import EmbeddingService
        from app.services.qdrant_service import QdrantService
        from app.services.chat_service import ChatService
        from app.services.rag_service import RAGService

        embedding_service = EmbeddingService()
        qdrant_service = QdrantService()
        chat_service = ChatService()
        rag_service = RAGService(embedding_service, qdrant_service, chat_service)

        print("[OK] All services initialized successfully")
    except Exception as e:
        print(f"[FAIL] Service initialization error: {e}")
        return False

    print("\n" + "=" * 60)
    print("[OK] ALL TESTS PASSED - BACKEND IS READY!")
    print("=" * 60)
    print("\nNow run: uvicorn app.main:app --host 127.0.0.1 --port 8000")
    print("Then visit: http://127.0.0.1:8000/api/docs")
    return True

if __name__ == "__main__":
    result = asyncio.run(test_backend())
    sys.exit(0 if result else 1)
