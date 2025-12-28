import pytest
import os
from unittest.mock import Mock, patch
from backend.services.qdrant import QdrantService
from backend.services.postgres import PostgresService
from backend.services.llm import LLMService


class TestQdrantService:
    """Unit tests for QdrantService"""

    @patch('backend.services.qdrant.QdrantClient')
    def test_init(self, mock_qdrant_client):
        """Test QdrantService initialization"""
        # Mock environment variables
        with patch.dict(os.environ, {
            'QDRANT_URL': 'http://localhost:6333',
            'QDRANT_API_KEY': 'test-key'
        }):
            service = QdrantService()

            # Verify client was created with correct parameters
            assert service.url == 'http://localhost:6333'
            assert service.api_key == 'test-key'

    @patch('backend.services.qdrant.QdrantClient')
    def test_check_connection_success(self, mock_qdrant_client):
        """Test Qdrant connection check"""
        mock_client = Mock()
        mock_qdrant_client.return_value = mock_client
        mock_client.get_collections.return_value = Mock()

        service = QdrantService()
        result = service.check_connection()

        assert result is True
        mock_client.get_collections.assert_called_once()

    @patch('backend.services.qdrant.QdrantClient')
    def test_check_connection_failure(self, mock_qdrant_client):
        """Test Qdrant connection check failure"""
        mock_client = Mock()
        mock_qdrant_client.return_value = mock_client
        mock_client.get_collections.side_effect = Exception("Connection failed")

        service = QdrantService()
        result = service.check_connection()

        assert result is False


class TestPostgresService:
    """Unit tests for PostgresService"""

    def test_init_missing_env_var(self):
        """Test PostgresService initialization without required env var"""
        # Remove any existing env var
        old_db_url = os.environ.get('NEON_DATABASE_URL')
        if 'NEON_DATABASE_URL' in os.environ:
            del os.environ['NEON_DATABASE_URL']

        try:
            with pytest.raises(ValueError, match="NEON_DATABASE_URL environment variable is required"):
                PostgresService()
        finally:
            # Restore original value
            if old_db_url is not None:
                os.environ['NEON_DATABASE_URL'] = old_db_url


class TestLLMService:
    """Unit tests for LLMService"""

    def test_init_missing_api_key(self):
        """Test LLMService initialization without required API key"""
        # Remove any existing env var
        old_api_key = os.environ.get('GEMINI_API_KEY')
        if 'GEMINI_API_KEY' in os.environ:
            del os.environ['GEMINI_API_KEY']

        try:
            with pytest.raises(ValueError, match="GEMINI_API_KEY environment variable is required"):
                LLMService()
        finally:
            # Restore original value
            if old_api_key is not None:
                os.environ['GEMINI_API_KEY'] = old_api_key