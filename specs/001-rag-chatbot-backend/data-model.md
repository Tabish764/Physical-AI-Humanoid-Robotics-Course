# Data Model: RAG Chatbot Backend API

This document outlines the key data entities and their attributes, derived from the feature specification (`spec.md`), that will be used in the RAG Chatbot Backend API.

## Entities

### 1. Content Segment

Represents a portion of the textbook content that has been processed for retrieval and vector embedding.

*   **Attributes**:
    *   `text` (string, **required**): The actual content of the segment.
    *   `source_identifier` (string, **required**): A unique reference to the original source file (e.g., `module1-ros2/ros2-architecture.md`).
    *   `chapter_or_module` (string, **required**): Categorization based on content location (e.g., `module1-ros2`).
    *   `segment_index` (integer, **required**): Position within the original document (0-indexed).
    *   `embedding` (list of floats, **required**): Numerical representation (vector) of the `text` for similarity search. (Implicitly stored by Qdrant).

*   **Storage**: Stored in the Qdrant Vector Database.
    *   The `text` attribute is converted into a vector (`embedding`) and stored in the `physical_ai_book` collection.
    *   `source_identifier`, `chapter_or_module`, `segment_index`, and `content` (original text) are stored as associated metadata (payload) in Qdrant.

*   **Validation Rules**:
    *   `text` must not be empty.
    *   `source_identifier` must be a valid, non-empty string representing a file path.
    *   `chapter_or_module` must be a valid, non-empty string.
    *   `segment_index` must be a non-negative integer.

### 2. Conversation Record

Represents a historical exchange between a user and the chatbot, stored for auditing and retrieval.

*   **Attributes**:
    *   `record_id` (UUID or integer, **primary key**, auto-generated):
    *   `timestamp` (datetime, **required**): Time of interaction (e.g., `DEFAULT NOW()`).
    *   `user_query` (string, **required**): The question or selected text submitted by the user.
    *   `chatbot_response` (string, **required**): The answer provided by the system.
    *   `cited_sources` (list of strings, **required**): A list of `source_identifier`s referenced in the `chatbot_response`. Can be empty if no sources are cited (e.g., for selected text explanations).
    *   `interaction_session_id` (string, **optional**): An identifier for grouping related interactions within a session (for future use).

*   **Storage**: Stored in Neon Serverless Postgres.
    *   Table: `chat_history`.
    *   Schema adherence to the defined attributes.

*   **Validation Rules**:
    *   `user_query` must not be empty.
    *   `chatbot_response` must not be empty.
    *   `cited_sources` should be a list of strings; can be empty.

