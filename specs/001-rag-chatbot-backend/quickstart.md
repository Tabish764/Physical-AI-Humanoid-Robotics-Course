# Quickstart Guide: RAG Chatbot Backend API

This guide provides a quick overview of how to set up and run the RAG Chatbot Backend API.

## 1. Prerequisites

Before you begin, ensure you have the following installed:

*   **Python 3.10+**
*   **Poetry** (or pip for dependency management)
*   **Docker Desktop** (for local Qdrant and Neon Postgres, if not using cloud services)
*   **API Keys** for:
    *   Google Gemini API
    *   Qdrant Cloud (if not running locally)
    *   Neon Serverless Postgres (if not running locally)

## 2. Environment Setup

1.  **Clone the repository**:
    ```bash
    git clone <repository_url>
    cd ai-native-book/backend
    ```

2.  **Create and configure `.env` file**:
    Copy the `.env.example` (if available) or create a new `.env` file in the `backend/` directory with your API keys and service URLs:
    ```
    GEMINI_API_KEY=your_gemini_key
    OPENAI_BASE_URL=https://generativelanguage.googleapis.com/v1beta/openai/
    QDRANT_URL=https://your-cluster.qdrant.io  # or http://localhost:6333 for local Docker
    QDRANT_API_KEY=your_qdrant_key             # or empty for local Docker
    NEON_DATABASE_URL=postgresql://user:pass@host/db # or postgresql://user:pass@localhost:5432/db for local Docker
    FRONTEND_URL=http://localhost:3000         # Your Docusaurus frontend URL
    ```

3.  **Install Python Dependencies**:
    ```bash
    poetry install
    # or using pip:
    # pip install -r requirements.txt
    ```

## 3. Book Content Embedding

Before running the API, you need to embed the book content into the Qdrant vector database.

1.  Ensure your `.env` file is configured with `QDRANT_URL` and `QDRANT_API_KEY`.
2.  Run the embedding script:
    ```bash
    poetry run python embed_book.py
    # or using pip:
    # python embed_book.py
    ```
    This process will read markdown files from the `../docs/` directory, chunk them, generate embeddings using the Gemini API, and store them in Qdrant.

## 4. Running the Backend API

1.  Ensure your `.env` file is configured with all necessary API keys and database URLs.
2.  Start the FastAPI application:
    ```bash
    poetry run uvicorn main:app --host 0.0.0.0 --port 8000 --reload
    # or using pip:
    # uvicorn main:app --host 0.0.0.0 --port 8000 --reload
    ```
    The API will be accessible at `http://localhost:8000`.

## 5. API Endpoints

*   **POST /api/chat**: Ask a question about the book content.
*   **POST /api/chat-selected**: Get an explanation for selected text.
*   **GET /api/health**: Check the health status of the API and its dependencies.

## 6. Testing (Optional)

Run the tests using `pytest`:

```bash
poetry run pytest
# or using pip:
# pytest
```
