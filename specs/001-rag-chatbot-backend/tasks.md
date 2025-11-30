# Feature Tasks: RAG Chatbot Backend API

This document outlines the tasks for implementing the RAG Chatbot Backend API, organized by user story and following the specified checklist format. Each task is designed to be immediately executable by an LLM.

## Phase 1: Setup

These tasks involve the initial project setup and environment configuration.

- [ ] T001 Create `backend/` directory and basic Python project structure
- [ ] T002 Create `backend/requirements.txt` with initial dependencies (fastapi, uvicorn, qdrant-client, openai, psycopg2-binary, python-dotenv, pydantic, markdown)
- [ ] T003 Create `backend/.env.example` file based on environment variables in plan.md
- [ ] T004 Create `backend/.gitignore` to ignore `.env` and `__pycache__`
- [ ] T005 Create `backend/README.md` with setup instructions from quickstart.md

## Phase 2: Foundational

These tasks establish the core infrastructure and connections required before implementing specific user stories.

- [ ] T006 Initialize FastAPI application in `backend/main.py`
- [ ] T007 Implement CORS middleware in `backend/main.py` using `FRONTEND_URL` environment variable
- [ ] T008 Implement basic structured logging configuration in `backend/utils/logger.py`
- [ ] T009 Create Qdrant client utility and connection function in `backend/services/qdrant.py`
- [ ] T010 Create Neon Postgres connection utility and define `chat_history` table schema in `backend/services/postgres.py`
- [ ] T011 Implement OpenAI SDK (Gemini) client utility with retry logic in `backend/services/llm.py`

## Phase 3: User Story 1 - Ask a Question about Book Content (Priority: P1)

A user wants to ask a question about the content of the Physical AI textbook and receive an accurate answer with cited sources.

**Independent Test**: A user can ask a question related to the textbook content (e.g., "What is ROS 2?"), and receive a coherent answer that includes relevant sections of the book as sources.

- [ ] T012 [P] [US1] Create `backend/models/content.py` for `ContentSegment` Pydantic model
- [ ] T013 [P] [US1] Create `backend/models/chat.py` for `ConversationRecord` Pydantic model
- [ ] T014 [US1] Implement `embed_book.py` script for content processing
    - Read markdown files recursively from `../docs/`
    - Strip frontmatter
    - Segment content into chunks (800-1200 chars, 100 char overlap)
    - Generate embeddings using OpenAI SDK (Gemini `text-embedding-004` model) via `backend/services/llm.py`
    - Store in Qdrant collection "physical_ai_book" with metadata via `backend/services/qdrant.py`
    - Handle rate limits with retry logic
    - Log progress
- [ ] T015 [US1] Implement `POST /api/chat` endpoint in `backend/main.py`
    - Receive `question` from request body (validated by Pydantic model)
    - Generate embedding for `question` using `backend/services/llm.py`
    - Query Qdrant for top 3 similar chunks via `backend/services/qdrant.py`
    - Extract text and `source_identifier`s from results
    - Construct prompt for Gemini based on retrieved context
    - Send prompt to OpenAI SDK (Gemini `gemini-1.5-flash` model) via `backend/services/llm.py`
    - Parse LLM response for answer
    - Save `ConversationRecord` to Neon Postgres via `backend/services/postgres.py`
    - Return JSON response with `answer` and `sources`

## Phase 4: User Story 2 - Get Explanation for Selected Text (Priority: P1)

A user wants to select a portion of text from the Physical AI textbook and ask the chatbot to explain it in more detail.

**Independent Test**: A user can select text from the content, request an explanation for it, and receive an answer based directly on the provided text, without external searches.

- [ ] T016 [P] [US2] Update `backend/models/chat.py` for `ChatSelectedRequest` Pydantic model (if needed)
- [ ] T017 [US2] Implement `POST /api/chat-selected` endpoint in `backend/main.py`
    - Receive `question` and `selected_text` from request body (validated by Pydantic model)
    - Construct prompt for Gemini based *only* on `selected_text`
    - Send prompt to OpenAI SDK (Gemini `gemini-1.5-flash` model) via `backend/services/llm.py`
    - Parse LLM response for answer
    - Save `ConversationRecord` (with empty sources) to Neon Postgres via `backend/services/postgres.py`
    - Return JSON response with `answer` and empty `sources` list

## Phase 5: User Story 3 - Check Backend Service Health (Priority: P2)

A developer or monitoring system wants to quickly verify the operational status of the content-aware API and its critical external dependencies.

**Independent Test**: A user can request a health check and receive a response indicating the connection status of the core data storage and retrieval services, and an overall service status.

- [ ] T018 [P] [US3] Create `backend/models/health.py` for `HealthStatus` Pydantic model
- [ ] T019 [US3] Implement `GET /api/health` endpoint in `backend/main.py`
    - Check Qdrant connectivity via `backend/services/qdrant.py`
    - Check Neon Postgres connectivity via `backend/services/postgres.py`
    - Return `HealthStatus` JSON response with overall `status` and individual service statuses (`qdrant`, `postgres`)

## Final Phase: Polish & Cross-Cutting Concerns

These tasks ensure the overall quality, robustness, and adherence to standards.

- [ ] T020 Implement comprehensive Pydantic models for all API request and response bodies.
- [ ] T021 Implement standardized error handling for all API endpoints with clear messages and appropriate HTTP status codes (400, 500, 503).
- [ ] T022 Ensure all environment variables are correctly loaded and used, and no secrets are hardcoded.
- [ ] T023 Add unit tests for `backend/services/qdrant.py`, `backend/services/postgres.py`, and `backend/services/llm.py`.
- [ ] T024 Add integration tests for `/api/chat`, `/api/chat-selected`, and `/api/health` endpoints using `pytest` and `httpx`.
- [ ] T025 Review and add comprehensive type hints to all functions and classes across the `backend/` codebase.

## Dependency Graph (User Story Completion Order)

1.  **Phase 1: Setup** (No dependencies)
2.  **Phase 2: Foundational** (Depends on Phase 1)
3.  **Phase 3: User Story 1 - Ask a Question about Book Content** (Depends on Phase 2)
4.  **Phase 4: User Story 2 - Get Explanation for Selected Text** (Depends on Phase 2)
5.  **Phase 5: User Story 3 - Check Backend Service Health** (Depends on Phase 2)
6.  **Final Phase: Polish & Cross-Cutting Concerns** (Depends on completion of all User Story phases)

## Parallel Execution Examples (within User Stories)

*   **User Story 1**: Tasks T012 and T013 can be done in parallel.
*   **User Story 2**: Task T016 can be done in parallel with T017's initial setup.
*   **User Story 3**: Task T018 can be done in parallel with T019's initial setup.

## Implementation Strategy

The implementation will follow an MVP-first approach, iteratively delivering functional user stories. Phase 1 and 2 will be completed first to establish a solid foundation. User Story 1 (Ask a Question) is P1 and will be the first fully implemented feature. User Story 2 (Explain Selected Text) is also P1 and can be developed in parallel or immediately after US1, leveraging shared LLM services. User Story 3 (Health Check) is P2 and will follow. The Final Phase will focus on refining the entire system, adding comprehensive tests, and ensuring adherence to quality gates.
