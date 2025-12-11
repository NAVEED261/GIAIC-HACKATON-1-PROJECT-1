-- Database Schema: RAG Chatbot Integration
-- Target Database: Neon Postgres (serverless PostgreSQL)
-- Migration Tool: Alembic
-- Created: 2025-12-11
-- Version: 1.0.0

-- Extension for UUID generation
CREATE EXTENSION IF NOT EXISTS "uuid-ossp";

-- =====================================================
-- Table: chat_sessions
-- Purpose: Track individual chat sessions
-- =====================================================

CREATE TABLE IF NOT EXISTS chat_sessions (
    -- Primary key
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),

    -- Client-generated session ID (stored in localStorage)
    session_id VARCHAR(255) UNIQUE NOT NULL,

    -- Timestamps
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW() NOT NULL,
    last_activity TIMESTAMP WITH TIME ZONE DEFAULT NOW() NOT NULL,

    -- Metadata (JSON)
    metadata JSONB DEFAULT '{}'::jsonb NOT NULL,

    -- Indexes
    CONSTRAINT session_id_not_empty CHECK (session_id <> '')
);

-- Indexes for chat_sessions
CREATE INDEX IF NOT EXISTS idx_chat_sessions_session_id
ON chat_sessions(session_id);

CREATE INDEX IF NOT EXISTS idx_chat_sessions_last_activity
ON chat_sessions(last_activity DESC);

-- =====================================================
-- Table: chat_messages
-- Purpose: Store chat history (user questions, assistant answers)
-- =====================================================

CREATE TABLE IF NOT EXISTS chat_messages (
    -- Primary key
    id BIGSERIAL PRIMARY KEY,

    -- Foreign key to chat_sessions
    session_id VARCHAR(255) NOT NULL,

    -- Message metadata
    role VARCHAR(50) NOT NULL CHECK (role IN ('user', 'assistant')),
    content TEXT NOT NULL,

    -- Assistant message metadata (NULL for user messages)
    sources JSONB DEFAULT '[]'::jsonb,
    confidence FLOAT CHECK (confidence IS NULL OR (confidence >= 0.0 AND confidence <= 1.0)),
    tokens_used INTEGER CHECK (tokens_used IS NULL OR tokens_used >= 0),

    -- Timestamp
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW() NOT NULL,

    -- Foreign key constraint
    CONSTRAINT fk_chat_messages_session
        FOREIGN KEY (session_id)
        REFERENCES chat_sessions(session_id)
        ON DELETE CASCADE,

    -- Constraints
    CONSTRAINT content_not_empty CHECK (content <> '')
);

-- Indexes for chat_messages
CREATE INDEX IF NOT EXISTS idx_chat_messages_session_id
ON chat_messages(session_id);

CREATE INDEX IF NOT EXISTS idx_chat_messages_created_at
ON chat_messages(created_at DESC);

-- =====================================================
-- Sample Data (for testing)
-- =====================================================

-- Insert sample session
INSERT INTO chat_sessions (session_id, metadata)
VALUES (
    'session-1702300800-abc123',
    '{"user_agent": "Mozilla/5.0", "referrer": "https://naveed261.github.io"}'::jsonb
) ON CONFLICT (session_id) DO NOTHING;

-- Insert sample user message
INSERT INTO chat_messages (session_id, role, content)
VALUES (
    'session-1702300800-abc123',
    'user',
    'What is ROS 2?'
) ON CONFLICT DO NOTHING;

-- Insert sample assistant message
INSERT INTO chat_messages (session_id, role, content, sources, confidence, tokens_used)
VALUES (
    'session-1702300800-abc123',
    'assistant',
    'ROS 2 is an open-source framework for robot software development...',
    '[{"chapter": "ROS 2 Fundamentals", "module": 1, "week": 3, "score": 0.92}]'::jsonb,
    0.92,
    1523
) ON CONFLICT DO NOTHING;

-- =====================================================
-- Alembic Migration Template
-- =====================================================

-- This SQL serves as reference for Alembic autogenerate.
-- Actual migration will be generated using:
-- alembic revision --autogenerate -m "Create chat tables"

/*
Alembic upgrade script template:

from alembic import op
import sqlalchemy as sa
from sqlalchemy.dialects.postgresql import UUID, JSONB

def upgrade():
    # Create chat_sessions table
    op.create_table(
        'chat_sessions',
        sa.Column('id', UUID(as_uuid=True), primary_key=True, server_default=sa.text('uuid_generate_v4()')),
        sa.Column('session_id', sa.String(255), unique=True, nullable=False),
        sa.Column('created_at', sa.DateTime(timezone=True), server_default=sa.text('NOW()'), nullable=False),
        sa.Column('last_activity', sa.DateTime(timezone=True), server_default=sa.text('NOW()'), nullable=False),
        sa.Column('metadata', JSONB, server_default=sa.text("'{}'::jsonb"), nullable=False),
        sa.CheckConstraint("session_id <> ''", name='session_id_not_empty')
    )

    op.create_index('idx_chat_sessions_session_id', 'chat_sessions', ['session_id'])
    op.create_index('idx_chat_sessions_last_activity', 'chat_sessions', ['last_activity'], postgresql_ops={'last_activity': 'DESC'})

    # Create chat_messages table
    op.create_table(
        'chat_messages',
        sa.Column('id', sa.BigInteger, primary_key=True, autoincrement=True),
        sa.Column('session_id', sa.String(255), nullable=False),
        sa.Column('role', sa.String(50), nullable=False),
        sa.Column('content', sa.Text, nullable=False),
        sa.Column('sources', JSONB, server_default=sa.text("'[]'::jsonb")),
        sa.Column('confidence', sa.Float),
        sa.Column('tokens_used', sa.Integer),
        sa.Column('created_at', sa.DateTime(timezone=True), server_default=sa.text('NOW()'), nullable=False),
        sa.ForeignKeyConstraint(['session_id'], ['chat_sessions.session_id'], ondelete='CASCADE'),
        sa.CheckConstraint("role IN ('user', 'assistant')", name='valid_role'),
        sa.CheckConstraint("confidence IS NULL OR (confidence >= 0.0 AND confidence <= 1.0)", name='valid_confidence'),
        sa.CheckConstraint("tokens_used IS NULL OR tokens_used >= 0", name='valid_tokens'),
        sa.CheckConstraint("content <> ''", name='content_not_empty')
    )

    op.create_index('idx_chat_messages_session_id', 'chat_messages', ['session_id'])
    op.create_index('idx_chat_messages_created_at', 'chat_messages', ['created_at'], postgresql_ops={'created_at': 'DESC'})

def downgrade():
    op.drop_table('chat_messages')
    op.drop_table('chat_sessions')
*/

-- =====================================================
-- Maintenance Queries
-- =====================================================

-- Get recent sessions (last 10)
-- SELECT session_id, created_at, last_activity
-- FROM chat_sessions
-- ORDER BY last_activity DESC
-- LIMIT 10;

-- Get chat history for a session
-- SELECT role, content, created_at
-- FROM chat_messages
-- WHERE session_id = 'session-1702300800-abc123'
-- ORDER BY created_at ASC;

-- Get average confidence score
-- SELECT AVG(confidence) as avg_confidence
-- FROM chat_messages
-- WHERE role = 'assistant' AND confidence IS NOT NULL;

-- Get total messages count
-- SELECT COUNT(*) as total_messages
-- FROM chat_messages;

-- Get messages per session
-- SELECT session_id, COUNT(*) as message_count
-- FROM chat_messages
-- GROUP BY session_id
-- ORDER BY message_count DESC;

-- Archive old sessions (>30 days inactive)
-- DELETE FROM chat_sessions
-- WHERE last_activity < NOW() - INTERVAL '30 days';

-- =====================================================
-- Performance Notes
-- =====================================================

-- 1. Index on session_id ensures fast session lookup
-- 2. Index on last_activity enables efficient recent sessions query
-- 3. Index on created_at enables chronological message retrieval
-- 4. CASCADE DELETE ensures orphaned messages are cleaned up
-- 5. JSONB type allows flexible metadata/sources storage
-- 6. Check constraints enforce data integrity at database level

-- =====================================================
-- Storage Estimates
-- =====================================================

-- Assumptions:
-- - Average message content: 200 characters
-- - Average sources JSON: 150 characters
-- - 10,000 messages total
--
-- Estimated storage:
-- chat_sessions: ~1KB per session × 1,000 sessions = 1MB
-- chat_messages: ~500 bytes per message × 10,000 messages = 5MB
-- Indexes: ~2MB
-- Total: ~8MB (well within Neon free tier 512MB limit)

-- =====================================================
-- Security Notes
-- =====================================================

-- 1. session_id is client-generated (no authentication in Phase 1)
-- 2. No PII stored (queries are anonymized)
-- 3. SSL/TLS enforced by Neon Postgres
-- 4. Input validation handled by Pydantic models in FastAPI
-- 5. Rate limiting enforced at application layer (not database)

-- END OF SCHEMA
