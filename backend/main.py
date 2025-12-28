from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
import os
from typing import List
from dotenv import load_dotenv
from models.chat import ChatRequest, ChatResponse, ConversationRecord, ChatSelectedRequest, ChatSelectedResponse
from models.health import HealthStatus
from services.qdrant import qdrant_service
from services.llm import llm_service
from services.postgres import postgres_service
from utils.logger import logger

# Load environment variables
load_dotenv()

# Initialize FastAPI app
app = FastAPI(title="RAG Chatbot Backend API", version="1.0.0")

# Add exception handlers for standardized error responses
@app.exception_handler(HTTPException)
async def http_exception_handler(request: Request, exc: HTTPException):
    return JSONResponse(
        status_code=exc.status_code,
        content={"error": exc.detail}
    )

@app.exception_handler(Exception)
async def general_exception_handler(request: Request, exc: Exception):
    logger.error(f"Unexpected error: {str(exc)}")
    return JSONResponse(
        status_code=500,
        content={"error": "Internal server error"}
    )

# Configure CORS middleware using FRONTEND_URL environment variable
frontend_url = os.getenv("FRONTEND_URL", "http://localhost:3000")
app.add_middleware(
    CORSMiddleware,
    allow_origins=[frontend_url],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/")
def read_root():
    return {"message": "RAG Chatbot Backend API"}

@app.post("/api/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest) -> ChatResponse:
    """
    Endpoint for asking a question about the book content.
    """
    try:
        # Validate input
        if not request.question or not request.question.strip():
            raise HTTPException(status_code=400, detail="Question cannot be empty")

        # Generate embedding for the question
        question_embedding = llm_service.generate_embedding(request.question)

        # Query Qdrant for top 3 similar chunks
        similar_chunks = qdrant_service.search_vectors(
            collection_name="physical_ai_book",
            query_vector=question_embedding,
            limit=3
        )

        # Extract text and source identifiers from results
        context_parts: List[str] = []
        sources: set = set()  # Use set to avoid duplicate sources

        for chunk in similar_chunks:
            payload = chunk.get('payload', {})
            text = payload.get('text', '')
            source_id = payload.get('source_identifier', '')

            if text:
                context_parts.append(text)
            if source_id:
                sources.add(source_id)

        # Combine the context
        context = "\n\n".join(context_parts)

        # Construct prompt for Gemini based on retrieved context
        prompt = f"""
        Context: {context}

        Question: {request.question}

        Please provide a comprehensive answer to the question based on the provided context.
        If the context doesn't contain information to answer the question, please say so.
        """

        # Send prompt to OpenAI SDK (Gemini gemini-1.5-flash model)
        answer = llm_service.generate_response(prompt)

        # Create ConversationRecord and save to Neon Postgres
        sources_list = list(sources)
        record_id, timestamp = postgres_service.save_conversation(
            user_query=request.question,
            chatbot_response=answer,
            cited_sources=sources_list
        )

        # Return JSON response with answer and sources
        return ChatResponse(answer=answer, sources=sources_list)

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logger.error(f"Error in chat endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail="Internal server error")

@app.post("/api/chat-selected", response_model=ChatSelectedResponse)
async def chat_selected_endpoint(request: ChatSelectedRequest) -> ChatSelectedResponse:
    """
    Endpoint for getting an explanation for selected text.
    """
    try:
        # Validate input
        if not request.question or not request.question.strip():
            raise HTTPException(status_code=400, detail="Question cannot be empty")
        if not request.selected_text or not request.selected_text.strip():
            raise HTTPException(status_code=400, detail="Selected text cannot be empty")

        # Construct prompt for Gemini based *only* on selected_text
        prompt = f"""
        Selected Text: {request.selected_text}

        Question: {request.question}

        Please provide a detailed explanation for the selected text in response to the question.
        Base your answer only on the provided selected text.
        """

        # Send prompt to OpenAI SDK (Gemini gemini-1.5-flash model)
        answer = llm_service.generate_response(prompt)

        # Save ConversationRecord (with empty sources) to Neon Postgres
        record_id, timestamp = postgres_service.save_conversation(
            user_query=f"Question: {request.question}\nSelected Text: {request.selected_text}",
            chatbot_response=answer,
            cited_sources=[]  # Empty sources list as specified
        )

        # Return JSON response with answer and empty sources list
        return ChatSelectedResponse(answer=answer, sources=[])

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logger.error(f"Error in chat-selected endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail="Internal server error")

@app.get("/api/health", response_model=HealthStatus)
async def health_check() -> HealthStatus:
    """
    Endpoint for checking the health status of the backend services.
    """
    try:
        # Check Qdrant connectivity
        qdrant_status: str = "connected" if qdrant_service.check_connection() else "disconnected"

        # Check Neon Postgres connectivity
        postgres_status: str = "connected" if postgres_service.check_connection() else "disconnected"

        # Determine overall status
        overall_status: str = "ok" if (qdrant_status == "connected" and postgres_status == "connected") else "degraded"

        # Return HealthStatus JSON response with overall status and individual service statuses
        return HealthStatus(
            status=overall_status,
            qdrant=qdrant_status,
            postgres=postgres_status
        )
    except Exception as e:
        logger.error(f"Error in health check endpoint: {str(e)}")
        # Return degraded status if there's an error checking health
        return HealthStatus(
            status="degraded",
            qdrant="disconnected",
            postgres="disconnected"
        )

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)