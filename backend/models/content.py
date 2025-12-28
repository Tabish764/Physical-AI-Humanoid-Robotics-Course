from pydantic import BaseModel
from typing import List, Optional
from datetime import datetime


class ContentSegment(BaseModel):
    """
    Pydantic model for representing a content segment from the textbook.
    """
    text: str
    source_identifier: str
    chapter_or_module: str
    segment_index: int
    embedding: Optional[List[float]] = None  # Optional since embedding might be generated separately

    class Config:
        # Allow extra fields for flexibility with Qdrant metadata
        extra = "allow"