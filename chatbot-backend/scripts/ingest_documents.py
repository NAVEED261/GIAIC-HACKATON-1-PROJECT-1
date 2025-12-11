"""
Document ingestion script for the Physical AI Textbook.

Reads markdown files, chunks content, generates embeddings, and stores in Qdrant.

Usage:
    python scripts/ingest_documents.py
"""

import asyncio
import os
import sys
from pathlib import Path
from typing import List, Dict, Any
import re

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.services.embedding_service import EmbeddingService
from app.services.qdrant_service import QdrantService
from app.core.config import settings
from app.core.logging import setup_logging, get_logger

setup_logging()
logger = get_logger(__name__)


class DocumentChunker:
    """Chunk documents for embedding and retrieval."""

    def __init__(
        self,
        chunk_size: int = settings.CHUNK_SIZE,
        overlap: int = settings.CHUNK_OVERLAP
    ):
        """
        Initialize chunker with size and overlap.

        Args:
            chunk_size: Target chunk size in tokens (approximate)
            overlap: Overlap between chunks in tokens
        """
        self.chunk_size = chunk_size
        self.overlap = overlap

    def chunk_text(self, text: str) -> List[str]:
        """
        Chunk text into overlapping segments.

        Args:
            text: Input text to chunk

        Returns:
            List of text chunks
        """
        # Simple word-based chunking (approximates tokens)
        words = text.split()
        chunks = []

        # Rough approximation: 1 token ≈ 0.75 words
        words_per_chunk = int(self.chunk_size * 0.75)
        words_overlap = int(self.overlap * 0.75)

        for i in range(0, len(words), words_per_chunk - words_overlap):
            chunk_words = words[i:i + words_per_chunk]
            chunk = " ".join(chunk_words)

            if chunk.strip():
                chunks.append(chunk)

            # Break if we've reached the end
            if i + words_per_chunk >= len(words):
                break

        return chunks


class DocumentIngester:
    """Ingest documents from Physical AI Textbook."""

    def __init__(
        self,
        embedding_service: EmbeddingService,
        qdrant_service: QdrantService,
        chunker: DocumentChunker
    ):
        """
        Initialize ingester with services.

        Args:
            embedding_service: Service for generating embeddings
            qdrant_service: Service for storing vectors
            chunker: Document chunker
        """
        self.embedding_service = embedding_service
        self.qdrant_service = qdrant_service
        self.chunker = chunker

    async def ingest_directory(self, docs_path: Path) -> None:
        """
        Ingest all markdown files from documentation directory.

        Args:
            docs_path: Path to docs directory
        """
        logger.info(f"Starting document ingestion from: {docs_path}")

        # Find all markdown files
        md_files = list(docs_path.rglob("*.md"))

        if not md_files:
            logger.warning(f"No markdown files found in {docs_path}")
            return

        logger.info(f"Found {len(md_files)} markdown files")

        # Create Qdrant collection
        await self.qdrant_service.create_collection()

        # Process files
        all_chunks = []
        all_payloads = []

        for md_file in md_files:
            try:
                logger.info(f"Processing: {md_file.relative_to(docs_path)}")

                chunks, payloads = await self._process_file(md_file, docs_path)

                all_chunks.extend(chunks)
                all_payloads.extend(payloads)

            except Exception as e:
                logger.error(f"Failed to process {md_file}: {e}")
                continue

        # Generate embeddings in batches
        logger.info(f"Generating embeddings for {len(all_chunks)} chunks...")

        batch_size = 100
        for i in range(0, len(all_chunks), batch_size):
            batch_chunks = all_chunks[i:i + batch_size]
            batch_payloads = all_payloads[i:i + batch_size]

            logger.info(f"Processing batch {i // batch_size + 1}/{(len(all_chunks) + batch_size - 1) // batch_size}")

            # Generate embeddings
            embeddings = await self.embedding_service.embed_batch(batch_chunks)

            # Store in Qdrant
            await self.qdrant_service.upsert_points(embeddings, batch_payloads)

        logger.info(f"Ingestion complete! Stored {len(all_chunks)} chunks")

        # Show collection info
        info = self.qdrant_service.get_collection_info()
        logger.info(f"Collection info: {info}")

    async def _process_file(
        self,
        file_path: Path,
        base_path: Path
    ) -> tuple[List[str], List[Dict[str, Any]]]:
        """
        Process a single markdown file.

        Args:
            file_path: Path to markdown file
            base_path: Base documentation path

        Returns:
            Tuple of (chunks, payloads)
        """
        # Read file
        content = file_path.read_text(encoding="utf-8")

        # Extract frontmatter metadata
        metadata = self._extract_metadata(content)

        # Remove frontmatter and clean content
        content = self._clean_content(content)

        # Chunk content
        chunks = self.chunker.chunk_text(content)

        # Create payloads
        relative_path = file_path.relative_to(base_path)
        payloads = [
            {
                "text": chunk,
                "chapter": metadata.get("title", file_path.stem),
                "module": metadata.get("module"),
                "week": metadata.get("week"),
                "file_path": str(relative_path)
            }
            for chunk in chunks
        ]

        return chunks, payloads

    def _extract_metadata(self, content: str) -> Dict[str, Any]:
        """
        Extract frontmatter metadata from markdown.

        Args:
            content: Markdown content

        Returns:
            Dictionary of metadata
        """
        metadata = {}

        # Match YAML frontmatter
        frontmatter_pattern = r"^---\s*\n(.*?)\n---"
        match = re.match(frontmatter_pattern, content, re.DOTALL)

        if match:
            frontmatter = match.group(1)

            # Extract title
            title_match = re.search(r'title:\s*["\']?(.*?)["\']?\s*$', frontmatter, re.MULTILINE)
            if title_match:
                metadata["title"] = title_match.group(1).strip('"\'')

            # Extract week
            week_match = re.search(r'week:\s*(\d+)', frontmatter)
            if week_match:
                metadata["week"] = int(week_match.group(1))

            # Extract module
            module_match = re.search(r'module:\s*(\d+)', frontmatter)
            if module_match:
                metadata["module"] = int(module_match.group(1))

        return metadata

    def _clean_content(self, content: str) -> str:
        """
        Clean markdown content for processing.

        Args:
            content: Raw markdown content

        Returns:
            Cleaned content
        """
        # Remove frontmatter
        content = re.sub(r"^---\s*\n.*?\n---\s*\n", "", content, flags=re.DOTALL)

        # Remove code blocks (keep code, remove backticks)
        content = re.sub(r"```[\w]*\n(.*?)\n```", r"\1", content, flags=re.DOTALL)

        # Remove inline code backticks
        content = re.sub(r"`([^`]+)`", r"\1", content)

        # Remove markdown links but keep text
        content = re.sub(r"\[([^\]]+)\]\([^\)]+\)", r"\1", content)

        # Remove image links
        content = re.sub(r"!\[([^\]]*)\]\([^\)]+\)", r"\1", content)

        # Remove HTML tags
        content = re.sub(r"<[^>]+>", "", content)

        # Remove extra whitespace
        content = re.sub(r"\n\s*\n", "\n\n", content)

        return content.strip()


async def main():
    """Main ingestion function."""
    logger.info("=" * 60)
    logger.info("Physical AI Textbook Document Ingestion")
    logger.info("=" * 60)

    # Initialize services
    embedding_service = EmbeddingService()
    qdrant_service = QdrantService()
    chunker = DocumentChunker()

    ingester = DocumentIngester(
        embedding_service=embedding_service,
        qdrant_service=qdrant_service,
        chunker=chunker
    )

    # Determine docs path
    # Assuming script runs from chatbot-backend/ and docs are in ../physical-ai-textbook/docs/
    script_dir = Path(__file__).parent.parent
    docs_path = script_dir.parent / "physical-ai-textbook" / "docs"

    if not docs_path.exists():
        logger.error(f"Documentation directory not found: {docs_path}")
        logger.info("Please ensure the Physical AI Textbook docs are available")
        logger.info("Expected path: physical-ai-textbook/docs/")
        return

    # Run ingestion
    try:
        await ingester.ingest_directory(docs_path)

        logger.info("=" * 60)
        logger.info("Ingestion complete!")
        logger.info("=" * 60)

    except Exception as e:
        logger.error(f"Ingestion failed: {e}")
        raise


if __name__ == "__main__":
    asyncio.run(main())
