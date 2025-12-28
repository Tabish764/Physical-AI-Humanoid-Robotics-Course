import os
from typing import List, Optional, Dict, Any
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams, Filter
from dotenv import load_dotenv
from utils.logger import logger

# Load environment variables
load_dotenv()

class QdrantService:
    """
    Service class for interacting with Qdrant vector database.
    """

    def __init__(self):
        self.url = os.getenv("QDRANT_URL", "http://localhost:6333")
        self.api_key = os.getenv("QDRANT_API_KEY")

        # Initialize Qdrant client
        if self.api_key:
            self.client = QdrantClient(
                url=self.url,
                api_key=self.api_key
            )
        else:
            # For local development without API key
            self.client = QdrantClient(url=self.url)

    def get_client(self) -> QdrantClient:
        """Get the Qdrant client instance."""
        return self.client

    def check_connection(self) -> bool:
        """
        Check if Qdrant is accessible.

        Returns:
            True if connection is successful, False otherwise
        """
        try:
            # Try to get collections list to verify connection
            self.client.get_collections()
            logger.info("Qdrant connection successful")
            return True
        except Exception as e:
            logger.error(f"Qdrant connection failed: {str(e)}")
            return False

    def create_collection(self, collection_name: str, vector_size: int = 768):
        """
        Create a collection in Qdrant if it doesn't exist.

        Args:
            collection_name: Name of the collection to create
            vector_size: Size of the vectors (default 768 for text-embedding-ada-002)
        """
        try:
            # Check if collection exists
            collections = self.client.get_collections()
            collection_names = [c.name for c in collections.collections]

            if collection_name not in collection_names:
                # Create collection with specified vector size
                self.client.create_collection(
                    collection_name=collection_name,
                    vectors_config=VectorParams(size=vector_size, distance=Distance.COSINE)
                )
                logger.info(f"Created Qdrant collection: {collection_name}")
            else:
                logger.info(f"Qdrant collection {collection_name} already exists")
        except Exception as e:
            logger.error(f"Failed to create collection {collection_name}: {str(e)}")
            raise

    def upsert_vectors(self, collection_name: str, points: List[Dict[str, Any]]):
        """
        Upsert vectors into the specified collection.

        Args:
            collection_name: Name of the collection
            points: List of points to upsert, each with id, vector, and payload
        """
        try:
            self.client.upsert(
                collection_name=collection_name,
                points=points
            )
            logger.info(f"Upserted {len(points)} vectors into collection {collection_name}")
        except Exception as e:
            logger.error(f"Failed to upsert vectors: {str(e)}")
            raise

    def search_vectors(self,
                   collection_name: str,
                   query_vector: List[float],
                   limit: int = 3,
                   filters: Optional[Filter] = None) -> List[Dict[str, Any]]:
        """
        Search for similar vectors in the collection.
        
        Args:
            collection_name: Name of the collection
            query_vector: Query vector to search for
            limit: Maximum number of results to return
            filters: Optional filters to apply
            
        Returns:
            List of dictionaries containing id, score, and payload
        """
        try:
            # Use query_points method with 'query' parameter (for qdrant-client 1.16.1)
            results = self.client.query_points(
                collection_name=collection_name,
                query=query_vector,  # Changed from query_vector to query
                query_filter=filters,
                limit=limit,
                with_payload=True
            )

            # Format results to return payload data
            formatted_results = []
            for point in results.points:
                formatted_results.append({
                    'id': point.id,
                    'score': point.score,
                    'payload': point.payload
                })

            logger.info(f"Found {len(formatted_results)} similar vectors in collection {collection_name}")
            return formatted_results
        except Exception as e:
            logger.error(f"Failed to search vectors: {str(e)}")
            raise


# Create a singleton instance
qdrant_service = QdrantService()

def get_qdrant_service() -> QdrantService:
    """Get the singleton Qdrant service instance."""
    return qdrant_service