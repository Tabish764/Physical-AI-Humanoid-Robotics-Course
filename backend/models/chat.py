from pydantic import BaseModel
from typing import List, Optional
from datetime import datetime
import uuid


class ConversationRecord(BaseModel):
    """
    Pydantic model for representing a conversation record in chat history.
    """
    record_id: Optional[uuid.UUID] = None
    timestamp: Optional[datetime] = None
    user_query: str
    chatbot_response: str
    cited_sources: List[str] = []
    interaction_session_id: Optional[str] = None


class ChatRequest(BaseModel):
    """
    Pydantic model for chat request validation.
    """
    question: str


class ChatResponse(BaseModel):
    """
    Pydantic model for chat response validation.
    """
    answer: str
    sources: List[str]


class ChatSelectedRequest(BaseModel):
    """
    Pydantic model for chat-selected request validation.
    """
    question: str
    selected_text: str


class ChatSelectedResponse(BaseModel):
    """
    Pydantic model for chat-selected response validation.
    """
    answer: str
    sources: List[str] = []


class HealthStatus(BaseModel):
    """
    Pydantic model for health status response.
    """
    status: str
    qdrant: str
    postgres: str