"""Initial schema for chat sessions and messages

Revision ID: 001_initial_schema
Revises:
Create Date: 2025-12-11

"""
from typing import Sequence, Union

from alembic import op
import sqlalchemy as sa
from sqlalchemy.dialects import postgresql

# revision identifiers, used by Alembic.
revision: str = '001_initial_schema'
down_revision: Union[str, None] = None
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    """Create chat_sessions and chat_messages tables."""

    # Enable uuid-ossp extension for UUID generation
    op.execute('CREATE EXTENSION IF NOT EXISTS "uuid-ossp"')

    # Create chat_sessions table
    op.create_table(
        'chat_sessions',
        sa.Column('id', postgresql.UUID(as_uuid=True), server_default=sa.text('uuid_generate_v4()'), nullable=False),
        sa.Column('session_id', sa.String(length=255), nullable=False),
        sa.Column('created_at', sa.DateTime(timezone=True), server_default=sa.text('now()'), nullable=False),
        sa.Column('last_activity', sa.DateTime(timezone=True), server_default=sa.text('now()'), nullable=False),
        sa.Column('metadata', postgresql.JSONB(astext_type=sa.Text()), server_default='{}', nullable=False),
        sa.PrimaryKeyConstraint('id'),
        sa.UniqueConstraint('session_id')
    )

    # Create index on session_id for faster lookups
    op.create_index(
        'idx_chat_sessions_session_id',
        'chat_sessions',
        ['session_id']
    )

    # Create chat_messages table
    op.create_table(
        'chat_messages',
        sa.Column('id', sa.BigInteger(), autoincrement=True, nullable=False),
        sa.Column('session_id', sa.String(length=255), nullable=False),
        sa.Column('role', sa.String(length=50), nullable=False),
        sa.Column('content', sa.Text(), nullable=False),
        sa.Column('sources', postgresql.JSONB(astext_type=sa.Text()), server_default='[]', nullable=False),
        sa.Column('confidence', sa.Float(), nullable=True),
        sa.Column('tokens_used', sa.Integer(), nullable=True),
        sa.Column('created_at', sa.DateTime(timezone=True), server_default=sa.text('now()'), nullable=False),
        sa.CheckConstraint("role IN ('user', 'assistant')", name='chat_messages_role_check'),
        sa.CheckConstraint('confidence >= 0.0 AND confidence <= 1.0', name='chat_messages_confidence_check'),
        sa.ForeignKeyConstraint(['session_id'], ['chat_sessions.session_id'], ondelete='CASCADE'),
        sa.PrimaryKeyConstraint('id')
    )

    # Create indexes for chat_messages
    op.create_index(
        'idx_chat_messages_session_id',
        'chat_messages',
        ['session_id']
    )

    op.create_index(
        'idx_chat_messages_created_at',
        'chat_messages',
        ['created_at']
    )


def downgrade() -> None:
    """Drop chat_messages and chat_sessions tables."""

    # Drop indexes first
    op.drop_index('idx_chat_messages_created_at', table_name='chat_messages')
    op.drop_index('idx_chat_messages_session_id', table_name='chat_messages')

    # Drop chat_messages table
    op.drop_table('chat_messages')

    # Drop index and table for chat_sessions
    op.drop_index('idx_chat_sessions_session_id', table_name='chat_sessions')
    op.drop_table('chat_sessions')

    # Drop uuid-ossp extension (optional, may be used by other tables)
    # op.execute('DROP EXTENSION IF EXISTS "uuid-ossp"')
