from pydantic import BaseModel
from typing import List, Optional
from datetime import datetime
import uuid


class HealthStatus(BaseModel):
    """
    Pydantic model for health status response.
    """
    status: str
    qdrant: str
    postgres: str