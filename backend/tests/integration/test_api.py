import pytest
from fastapi.testclient import TestClient
from backend.main import app


@pytest.fixture
def client():
    """Create a test client for the API"""
    return TestClient(app)


def test_root_endpoint(client):
    """Test the root endpoint"""
    response = client.get("/")
    assert response.status_code == 200
    assert "message" in response.json()
    assert response.json()["message"] == "RAG Chatbot Backend API"


def test_health_endpoint(client):
    """Test the health endpoint"""
    response = client.get("/api/health")
    assert response.status_code == 200

    data = response.json()
    assert "status" in data
    assert "qdrant" in data
    assert "postgres" in data
    # Note: Actual status depends on whether services are running
    # This test will pass regardless of actual service status


def test_chat_endpoint_bad_request(client):
    """Test the chat endpoint with invalid input"""
    # Test with empty request
    response = client.post("/api/chat", json={})
    assert response.status_code == 422  # Validation error

    # Test with empty question
    response = client.post("/api/chat", json={"question": ""})
    assert response.status_code == 400
    assert "error" in response.json()


def test_chat_selected_endpoint_bad_request(client):
    """Test the chat-selected endpoint with invalid input"""
    # Test with empty request
    response = client.post("/api/chat-selected", json={})
    assert response.status_code == 422  # Validation error

    # Test with empty question
    response = client.post("/api/chat-selected", json={"question": "", "selected_text": "test"})
    assert response.status_code == 400
    assert "error" in response.json()

    # Test with empty selected_text
    response = client.post("/api/chat-selected", json={"question": "test", "selected_text": ""})
    assert response.status_code == 400
    assert "error" in response.json()


# Note: Actual functional tests for chat and chat-selected endpoints would require
# mocked services (Qdrant, LLM, Postgres) to avoid external dependencies
# These are more complex integration tests that would be implemented in a full production setup