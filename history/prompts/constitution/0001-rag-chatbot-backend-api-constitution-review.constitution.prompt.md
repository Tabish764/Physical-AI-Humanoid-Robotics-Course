---
ID: 0001
TITLE: RAG Chatbot Backend API Constitution Review
STAGE: constitution
DATE_ISO: 2025-11-29
SURFACE: agent
MODEL: claude-sonnet-4-5-20250929
FEATURE: none
BRANCH: 1-ai-robotics-textbook
USER: user
COMMAND: /sp.constitution
LABELS: ["backend","fastapi","rag","qdrant","neon","gemini","openai-sdk","constitution"]
LINKS:
  SPEC: null
  TICKET: null
  ADR: null
  PR: null
FILES_YAML:
  - backend/main.py
  - backend/embed_book.py
  - backend/requirements.txt
  - backend/.env
  - backend/.gitignore
  - backend/README.md
TESTS_YAML: []
---
### User Prompt:
```
/sp.constitution

Project: RAG Chatbot Backend API using FastAPI, Qdrant Vector Database, Neon Postgres, and OpenAI SDK with Gemini API

Core principles:
- Clean REST API architecture with FastAPI
- Vector similarity search using Qdrant for intelligent book content retrieval
- OpenAI Python SDK configured to use Google Gemini API (OpenAI-compatible interface)
- Persistent chat history in Neon Serverless Postgres
- One-time book embedding setup script
- CORS-enabled for Docusaurus frontend integration

Key standards:
- Python 3.10+ with type hints on all functions
- Pydantic models for request/response validation
- Environment-based configuration using python-dotenv
- Proper error handling with descriptive HTTP status codes
- Structured logging for debugging
- RESTful endpoint design
- Rate limiting awareness (Gemini free tier: 10 RPM)

Project Structure:
backend/
├── main.py              # FastAPI application with chat endpoints
├── embed_book.py        # One-time script to embed book content into Qdrant
├── requirements.txt     # Python dependencies
├── .env                 # Environment variables (API keys, database URLs)
├── .gitignore          # Ignore .env and __pycache__
└── README.md           # Setup instructions

Technical Stack:
- FastAPI for REST API server
- Uvicorn as ASGI server
- Qdrant Cloud (free tier 1GB) for vector database
- Neon Serverless Postgres (free tier) for chat history
- OpenAI Python SDK configured with Gemini API endpoint
- Google Gemini API key used through OpenAI-compatible interface
- Python packages: fastapi, uvicorn, qdrant-client, openai, psycopg2-binary, python-dotenv, pydantic, markdown

OpenAI SDK Configuration with Gemini:
- Use OpenAI Python SDK: from openai import OpenAI
- Configure custom base_url for Gemini API endpoint
- Use Gemini API key as api_key parameter
- Models: Use Gemini model names (gemini-1.5-flash, text-embedding-004)
- Example client initialization:
  client = OpenAI(
      api_key=GEMINI_API_KEY,
      base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
  )

API Endpoints:
- POST /api/chat
  Request: {"question": "What is ROS 2?"}
  Response: {"answer": "ROS 2 is...", "sources": ["module1-ros2/ros2-architecture.md"]}

- POST /api/chat-selected
  Request: {"question": "Explain this", "selected_text": "ROS 2 is a middleware..."}
  Response: {"answer": "This text explains...", "sources": []}

- GET /api/health
  Response: {"status": "ok", "qdrant": "connected", "postgres": "connected"}

Book Embedding Process (embed_book.py):
- Walk through ../docs/ directory recursively
- Read all .md files (skip non-markdown files)
- Strip frontmatter (--- sections at top)
- Chunk content into 800-1200 character segments with 100 char overlap
- Generate embeddings using OpenAI SDK with Gemini endpoint (text-embedding-004 model)
- Store in Qdrant collection "physical_ai_book" with metadata:
  * file_path: relative path from docs/
  * module: extracted from folder name
  * chunk_index: position in document
  * content: original text chunk
- Handle rate limits with retry logic (exponential backoff)
- Log progress (e.g., "Embedded 25/100 chunks")

Chat Endpoint Logic (main.py):
1. Receive user question
2. Generate embedding for question using OpenAI SDK (Gemini endpoint)
3. Query Qdrant for top 3 similar chunks (cosine similarity)
4. Extract text and sources from results
5. Construct prompt: "Based on this context from a Physical AI textbook:

[chunks]

Answer this question: [question]"
6. Send to OpenAI SDK chat.completions.create() using gemini-1.5-flash model
7. Parse response
8. Save interaction to Neon Postgres (timestamp, question, answer, sources)
9. Return JSON response

Selected Text Endpoint Logic:
- Use provided selected_text as context instead of Qdrant search
- Construct prompt: "Based on this selected text:

[selected_text]

Answer: [question]"
- Send to OpenAI SDK with Gemini endpoint
- Return answer (no Qdrant search needed)

Qdrant Configuration:
- Collection name: "physical_ai_book"
- Vector size: 768 (Gemini text-embedding-004 dimension)
- Distance metric: Cosine similarity
- Payload schema: {file_path: str, module: str, content: str, chunk_index: int}

Neon Postgres Schema:
Table: chat_history
- id: SERIAL PRIMARY KEY
- timestamp: TIMESTAMP DEFAULT NOW()
- question: TEXT NOT NULL
- answer: TEXT NOT NULL
- sources: TEXT[] (array of file paths)
- session_id: VARCHAR(255) (optional for future use)

Environment Variables (.env):
- GEMINI_API_KEY=your_gemini_key
- OPENAI_BASE_URL=https://generativelanguage.googleapis.com/v1beta/openai/
- QDRANT_URL=https://your-cluster.qdrant.io
- QDRANT_API_KEY=your_qdrant_key
- NEON_DATABASE_URL=postgresql://user:pass@host/db
- FRONTEND_URL=http://localhost:3000 (for CORS)

OpenAI SDK Usage Pattern:
# Initialize client
from openai import OpenAI
client = OpenAI(
    api_key=os.getenv("GEMINI_API_KEY"),
    base_url=os.getenv("OPENAI_BASE_URL")
)

# For embeddings
response = client.embeddings.create(
    model="text-embedding-004",
    input="text to embed"
)

# For chat
response = client.chat.completions.create(
    model="gemini-1.5-flash",
    messages=[{"role": "user", "content": "prompt"}]
)

CORS Configuration:
- Allow origin: FRONTEND_URL from environment
- Allow methods: GET, POST, OPTIONS
- Allow headers: Content-Type, Authorization
- Allow credentials: True

Error Handling:
- 400 Bad Request: Invalid input (missing question)
- 500 Internal Server Error: API failures (OpenAI/Gemini, Qdrant, Postgres)
- 503 Service Unavailable: Rate limit exceeded
- All errors return JSON: {"error": "descriptive message"}

Logging:
- Log all API requests (endpoint, timestamp)
- Log Qdrant queries (search results count)
- Log OpenAI SDK calls (model, prompt length, response length)
- Log database operations (insert success/failure)
- Use Python logging module with INFO level

Success Criteria:
- embed_book.py successfully processes all markdown files in ../docs/
- All book content stored in Qdrant with valid embeddings
- POST /api/chat returns relevant answers with source citations
- POST /api/chat-selected handles selected text queries
- Chat history saved to Neon Postgres correctly
- Health endpoint returns 200 OK when services are reachable
- CORS allows requests from Docusaurus frontend
- Response time under 5 seconds for chat queries
- Handles Gemini rate limits gracefully (retry logic)
- No crashes on invalid input
- OpenAI SDK successfully configured with Gemini endpoint

Quality Gates:
- All functions have type hints
- Pydantic models validate all API inputs/outputs
- No hardcoded secrets (all in .env)
- Code follows PEP 8 style
- README includes clear setup instructions
- .gitignore excludes .env and __pycache__

Constraints:
- Free tier limits: Qdrant 1GB, Neon 0.5GB, Gemini 10 RPM
- Must read book from ../docs/ directory (relative path)
- No user authentication for MVP
- Stateless API (no session management in backend)
- Must use OpenAI Python SDK (not google-generativeai)
- Deploy to free hosting (Render, Railway, or run locally)

Non-Goals:
- User authentication (bonus feature, not MVP)
- Real-time streaming responses
- Multi-language support (handled separately in frontend)
- Conversation context (each question is independent)
- Fine-tuning custom models
- Using native Google SDK (must use OpenAI SDK)
```
### Assistant Response:
I have received and reviewed the constitution for your RAG Chatbot Backend API. This document clearly outlines the requirements, technical stack, API endpoints, and development guidelines for the project.

**Project Overview:**
The project aims to build a RAG Chatbot Backend API using FastAPI, Qdrant Vector Database, Neon Postgres, and the OpenAI SDK configured for the Gemini API.

**Core Principles Confirmed:**
*   Clean REST API architecture with FastAPI.
*   Vector similarity search using Qdrant for intelligent book content retrieval.
*   OpenAI Python SDK configured to use Google Gemini API.
*   Persistent chat history in Neon Serverless Postgres.
*   One-time book embedding setup script.
*   CORS-enabled for Docusaurus frontend integration.

**Key Standards (Invariants):**
*   Python 3.10+ with type hints on all functions.
*   Pydantic models for request/response validation.
*   Environment-based configuration using python-dotenv.
*   Proper error handling with descriptive HTTP status codes.
*   Structured logging for debugging.
*   RESTful endpoint design.
*   Rate limiting awareness.

**Project Structure and Technical Stack:**
I will proceed with the defined project structure and the specified technical stack, including FastAPI, Uvicorn, Qdrant Cloud, Neon Serverless Postgres, and the OpenAI Python SDK configured for Gemini.

**API Endpoints:**
The following API endpoints are clearly defined:
*   `POST /api/chat`
*   `POST /api/chat-selected`
*   `GET /api/health`

**Book Embedding Process (`embed_book.py`), Chat Endpoint Logic (`main.py`), Selected Text Endpoint Logic, Qdrant Configuration, Neon Postgres Schema, Environment Variables (`.env`), OpenAI SDK Usage Pattern, CORS Configuration, Error Handling, and Logging** are all well-defined and will be followed.

**Success Criteria and Quality Gates:**
The provided success criteria and quality gates will be used to verify the implementation.

**Constraints:**
*   Free tier limits: Qdrant 1GB, Neon 0.5GB, Gemini 10 RPM.
*   Must read book from `../docs/` directory (relative path).
*   No user authentication for MVP.
*   Stateless API (no session management in backend).
*   Must use OpenAI Python SDK (not `google-generativeai`).
*   Deploy to free hosting (Render, Railway, or run locally).

**Non-Goals:**
*   User authentication.
*   Real-time streaming responses.
*   Multi-language support.
*   Conversation context.
*   Fine-tuning custom models.
*   Using native Google SDK.
