import os
import psycopg2
from psycopg2.extras import RealDictCursor
from dotenv import load_dotenv
from utils.logger import logger
from typing import Optional, List, Dict, Any
import uuid
from datetime import datetime

# Load environment variables
load_dotenv()

class PostgresService:
    """
    Service class for interacting with Neon Postgres database.
    """

    def __init__(self):
        self.database_url = os.getenv("NEON_DATABASE_URL")
        if not self.database_url:
            raise ValueError("NEON_DATABASE_URL environment variable is required")

    def get_connection(self):
        """Get a connection to the Postgres database."""
        try:
            conn = psycopg2.connect(
                self.database_url,
                cursor_factory=RealDictCursor
            )
            return conn
        except Exception as e:
            logger.error(f"Failed to connect to Postgres: {str(e)}")
            raise

    def check_connection(self) -> bool:
        """
        Check if Neon Postgres is accessible.

        Returns:
            True if connection is successful, False otherwise
        """
        try:
            with self.get_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute("SELECT 1")
                    result = cur.fetchone()
                    if result:
                        logger.info("Postgres connection successful")
                        return True
        except Exception as e:
            logger.error(f"Postgres connection failed: {str(e)}")
            return False
        return False

    def create_chat_history_table(self):
        """
        Create the chat_history table if it doesn't exist.
        """
        try:
            with self.get_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute("""
                        CREATE TABLE IF NOT EXISTS chat_history (
                            record_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                            timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                            user_query TEXT NOT NULL,
                            chatbot_response TEXT NOT NULL,
                            cited_sources TEXT[] DEFAULT ARRAY[]::TEXT[],
                            interaction_session_id TEXT
                        )
                    """)
                    conn.commit()
                    logger.info("Chat history table created or already exists")
        except Exception as e:
            logger.error(f"Failed to create chat history table: {str(e)}")
            raise

    def save_conversation(self, user_query: str, chatbot_response: str, cited_sources: List[str] = None, interaction_session_id: str = None):
        """
        Save a conversation record to the database.

        Args:
            user_query: The user's question or input
            chatbot_response: The response from the chatbot
            cited_sources: List of sources cited in the response
            interaction_session_id: Optional session identifier
        """
        if cited_sources is None:
            cited_sources = []

        try:
            with self.get_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute("""
                        INSERT INTO chat_history (user_query, chatbot_response, cited_sources, interaction_session_id)
                        VALUES (%s, %s, %s, %s)
                        RETURNING record_id, timestamp
                    """, (user_query, chatbot_response, cited_sources, interaction_session_id))

                    result = cur.fetchone()
                    conn.commit()

                    record_id = result['record_id']
                    timestamp = result['timestamp']

                    logger.info(f"Saved conversation with record_id: {record_id}")
                    return record_id, timestamp
        except Exception as e:
            logger.error(f"Failed to save conversation: {str(e)}")
            raise

    def get_conversation(self, record_id: str) -> Optional[Dict[str, Any]]:
        """
        Retrieve a specific conversation by record ID.

        Args:
            record_id: The UUID of the conversation record

        Returns:
            Dictionary with conversation details or None if not found
        """
        try:
            with self.get_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute("""
                        SELECT record_id, timestamp, user_query, chatbot_response, cited_sources, interaction_session_id
                        FROM chat_history
                        WHERE record_id = %s
                    """, (record_id,))

                    result = cur.fetchone()

                    if result:
                        return {
                            'record_id': str(result['record_id']),
                            'timestamp': result['timestamp'],
                            'user_query': result['user_query'],
                            'chatbot_response': result['chatbot_response'],
                            'cited_sources': result['cited_sources'],
                            'interaction_session_id': result['interaction_session_id']
                        }
                    return None
        except Exception as e:
            logger.error(f"Failed to get conversation: {str(e)}")
            raise

    def get_recent_conversations(self, limit: int = 10) -> List[Dict[str, Any]]:
        """
        Retrieve recent conversations.

        Args:
            limit: Maximum number of records to return

        Returns:
            List of conversation records
        """
        try:
            with self.get_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute("""
                        SELECT record_id, timestamp, user_query, chatbot_response, cited_sources, interaction_session_id
                        FROM chat_history
                        ORDER BY timestamp DESC
                        LIMIT %s
                    """, (limit,))

                    results = cur.fetchall()

                    conversations = []
                    for result in results:
                        conversations.append({
                            'record_id': str(result['record_id']),
                            'timestamp': result['timestamp'],
                            'user_query': result['user_query'],
                            'chatbot_response': result['chatbot_response'],
                            'cited_sources': result['cited_sources'],
                            'interaction_session_id': result['interaction_session_id']
                        })

                    return conversations
        except Exception as e:
            logger.error(f"Failed to get recent conversations: {str(e)}")
            raise

# Create a singleton instance
postgres_service = PostgresService()

def get_postgres_service() -> PostgresService:
    """Get the singleton Postgres service instance."""
    return postgres_service