# RAG Chatbot Backend API

This is the backend API for the RAG Chatbot that allows users to ask questions about the Physical AI textbook content and receive accurate answers with cited sources.

## Prerequisites

Before you begin, ensure you have the following installed:

*   **Python 3.10+**
*   **Poetry** (or pip for dependency management)
*   **Docker Desktop** (for local Qdrant and Neon Postgres, if not using cloud services)
*   **API Keys** for:
    *   Google Gemini API
    *   Qdrant Cloud (if not running locally)
    *   Neon Serverless Postgres (if not running locally)

## Environment Setup

1.  **Create and configure `.env` file**:
    Copy the `.env.example` and create a new `.env` file in the `backend/` directory with your API keys and service URLs:

    ```bash
    cp .env.example .env
    ```

    Then edit `.env` to add your API keys and service URLs:

    ```
    GEMINI_API_KEY=your_gemini_key
    OPENAI_BASE_URL=https://generativelanguage.googleapis.com/v1beta/openai/
    QDRANT_URL=https://your-cluster.qdrant.io  # or http://localhost:6333 for local Docker
    QDRANT_API_KEY=your_qdrant_key             # or empty for local Docker
    NEON_DATABASE_URL=postgresql://user:pass@host/db # or postgresql://user:pass@localhost:5432/db for local Docker
    FRONTEND_URL=http://localhost:3000         # Your Docusaurus frontend URL
    ```

2.  **Install Python Dependencies**:

    ```bash
    pip install -r requirements.txt
    ```

## Book Content Embedding

Before running the API, you need to embed the book content into the Qdrant vector database.

1.  Ensure your `.env` file is configured with `QDRANT_URL` and `QDRANT_API_KEY`.
2.  Run the embedding script:

    ```bash
    python embed_book.py
    ```

    This process will read markdown files from the `../docs/` directory, chunk them, generate embeddings using the Gemini API, and store them in Qdrant.

## Running the Backend API

1.  Ensure your `.env` file is configured with all necessary API keys and database URLs.
2.  Start the FastAPI application:

    ```bash
    uvicorn main:app --host 0.0.0.0 --port 8000 --reload
    ```

    The API will be accessible at `http://localhost:8000`.

## API Endpoints

*   **POST /api/chat**: Ask a question about the book content.
*   **POST /api/chat-selected**: Get an explanation for selected text.
*   **GET /api/health**: Check the health status of the API and its dependencies.

## Testing (Optional)

Run the tests using `pytest`:

```bash
pytest
```