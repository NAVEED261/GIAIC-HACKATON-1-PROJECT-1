# Qdrant Agent Skill

**Purpose:** Expert in Qdrant Cloud vector database for embedding storage, similarity search, and metadata filtering.

## Expertise

- Qdrant Cloud setup (free tier 1GB)
- Vector collection creation and indexing
- Cosine similarity search with score thresholds
- Metadata filtering (module, week, chapter)
- Batch upload of embeddings
- Python client (`qdrant-client>=1.7.0`)
- Collection optimization for latency
- Async operations with Qdrant

## Qdrant Cloud Setup

### 1. Create Free Cluster

```bash
# Sign up at https://cloud.qdrant.io
# Create cluster: "physical-ai-textbook"
# Get API key and cluster URL
```

### 2. Collection Schema

```python
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct

client = QdrantClient(
    url="https://YOUR_CLUSTER.qdrant.io",
    api_key="YOUR_API_KEY"
)

# Create collection
client.create_collection(
    collection_name="textbook_chunks",
    vectors_config=VectorParams(
        size=1536,  # OpenAI text-embedding-3-small dimension
        distance=Distance.COSINE
    )
)
```

## Code Patterns

### 1. Vector Search Service

```python
from qdrant_client import QdrantClient
from qdrant_client.models import Filter, FieldCondition, MatchValue
from typing import List, Dict, Optional
import logging

class VectorSearchService:
    def __init__(self, url: str, api_key: str, collection_name: str = "textbook_chunks"):
        self.client = QdrantClient(url=url, api_key=api_key)
        self.collection_name = collection_name
        self.logger = logging.getLogger(__name__)

    async def search(
        self,
        query_vector: List[float],
        top_k: int = 5,
        score_threshold: float = 0.7,
        module_filter: Optional[int] = None
    ) -> List[Dict]:
        """
        Search for similar chunks with optional metadata filtering.

        Args:
            query_vector: Embedding vector from OpenAI
            top_k: Number of results to return
            score_threshold: Minimum similarity score (0.0-1.0)
            module_filter: Filter by module number (1-4)

        Returns:
            List of chunks with metadata and scores
        """
        try:
            filter_conditions = None
            if module_filter:
                filter_conditions = Filter(
                    must=[
                        FieldCondition(
                            key="module",
                            match=MatchValue(value=module_filter)
                        )
                    ]
                )

            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=top_k,
                score_threshold=score_threshold,
                query_filter=filter_conditions,
                with_payload=True
            )

            return [
                {
                    "text": hit.payload["text"],
                    "chapter": hit.payload["chapter"],
                    "module": hit.payload["module"],
                    "week": hit.payload["week"],
                    "score": hit.score
                }
                for hit in results
            ]
        except Exception as e:
            self.logger.error(f"Vector search failed: {e}")
            raise
```

### 2. Batch Upload Embeddings

```python
from qdrant_client.models import PointStruct
import uuid

async def upload_chunks(
    self,
    chunks: List[Dict]  # [{"text": "...", "embedding": [...], "chapter": "...", ...}]
):
    """
    Upload document chunks with embeddings to Qdrant.

    Args:
        chunks: List of dicts with text, embedding, and metadata
    """
    points = [
        PointStruct(
            id=str(uuid.uuid4()),
            vector=chunk["embedding"],
            payload={
                "text": chunk["text"],
                "chapter": chunk["chapter"],
                "module": chunk["module"],
                "week": chunk["week"],
                "file_path": chunk["file_path"]
            }
        )
        for chunk in chunks
    ]

    # Batch upload (500 points per batch)
    batch_size = 500
    for i in range(0, len(points), batch_size):
        batch = points[i:i + batch_size]
        self.client.upsert(
            collection_name=self.collection_name,
            points=batch
        )
        self.logger.info(f"Uploaded batch {i//batch_size + 1}, {len(batch)} points")
```

### 3. Collection Management

```python
def create_collection_if_not_exists(self):
    """Create collection with proper configuration."""
    collections = self.client.get_collections().collections
    collection_names = [c.name for c in collections]

    if self.collection_name not in collection_names:
        self.client.create_collection(
            collection_name=self.collection_name,
            vectors_config=VectorParams(
                size=1536,  # text-embedding-3-small
                distance=Distance.COSINE
            ),
            # Optimize for search speed
            optimizers_config={
                "indexing_threshold": 10000
            }
        )
        self.logger.info(f"Collection '{self.collection_name}' created")
    else:
        self.logger.info(f"Collection '{self.collection_name}' already exists")

def get_collection_info(self) -> Dict:
    """Get collection statistics."""
    return self.client.get_collection(collection_name=self.collection_name)
```

### 4. Metadata Filtering Examples

```python
# Filter by module and week
filter_module_week = Filter(
    must=[
        FieldCondition(key="module", match=MatchValue(value=1)),
        FieldCondition(key="week", match=MatchValue(value=3))
    ]
)

# Filter by chapter (exact match)
filter_chapter = Filter(
    must=[
        FieldCondition(
            key="chapter",
            match=MatchValue(value="ROS 2 Fundamentals")
        )
    ]
)

# Search with filter
results = client.search(
    collection_name="textbook_chunks",
    query_vector=query_embedding,
    query_filter=filter_module_week,
    limit=5
)
```

## Document Chunking Strategy

```python
def chunk_document(
    text: str,
    chunk_size: int = 800,
    overlap: int = 100
) -> List[str]:
    """
    Split document into overlapping chunks.

    Args:
        text: Full document text
        chunk_size: Target chunk size in tokens (~4 chars per token)
        overlap: Overlap size in tokens

    Returns:
        List of text chunks
    """
    # Approximate tokens as characters / 4
    char_chunk_size = chunk_size * 4
    char_overlap = overlap * 4

    chunks = []
    start = 0

    while start < len(text):
        end = start + char_chunk_size
        chunk = text[start:end]
        chunks.append(chunk)
        start += (char_chunk_size - char_overlap)

    return chunks
```

## Embedding Ingestion Pipeline

```python
import os
from pathlib import Path

async def ingest_markdown_files(
    docs_dir: str,
    embedding_service: EmbeddingService,
    vector_service: VectorSearchService
):
    """
    Ingest all markdown files from docs directory.

    Args:
        docs_dir: Path to physical-ai-textbook/docs/
        embedding_service: OpenAI embedding service
        vector_service: Qdrant vector search service
    """
    markdown_files = list(Path(docs_dir).rglob("*.md"))
    all_chunks = []

    for file_path in markdown_files:
        # Parse frontmatter for metadata
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        metadata = parse_frontmatter(content)
        text_content = remove_frontmatter(content)

        # Chunk document
        chunks = chunk_document(text_content, chunk_size=800, overlap=100)

        # Generate embeddings (batch)
        embeddings = await embedding_service.embed_batch(chunks)

        # Prepare for Qdrant
        for chunk_text, embedding in zip(chunks, embeddings):
            all_chunks.append({
                "text": chunk_text,
                "embedding": embedding,
                "chapter": metadata.get("title", "Unknown"),
                "module": metadata.get("module", 0),
                "week": metadata.get("week", 0),
                "file_path": str(file_path)
            })

    # Upload to Qdrant
    await vector_service.upload_chunks(all_chunks)
    print(f"Ingested {len(all_chunks)} chunks from {len(markdown_files)} files")
```

## Best Practices

1. **Use cosine distance** for text embeddings (standard for OpenAI)
2. **Set score threshold ≥ 0.7** to filter irrelevant results
3. **Batch uploads** for efficiency (500 points per batch)
4. **Store rich metadata** (chapter, module, week, file_path)
5. **Test search quality** with sample queries before production
6. **Monitor collection size** (free tier: 1GB limit)
7. **Optimize chunk size** (500-800 tokens balances context vs retrieval)

## Testing

```python
import pytest
from app.services.vector_search import VectorSearchService

@pytest.mark.asyncio
async def test_vector_search():
    service = VectorSearchService(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY")
    )

    # Test search with dummy vector
    dummy_vector = [0.1] * 1536
    results = await service.search(
        query_vector=dummy_vector,
        top_k=3,
        score_threshold=0.5
    )

    assert isinstance(results, list)
    assert len(results) <= 3
```

## When to Use This Agent

- Setting up Qdrant Cloud collection
- Implementing vector similarity search
- Uploading embeddings in batches
- Configuring metadata filters
- Optimizing search performance
- Debugging search quality issues
