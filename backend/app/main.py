import os
import logging
import time
from dotenv import load_dotenv
from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field
from typing import List, Dict, Any
from openai import OpenAI, RateLimitError
from qdrant_client import QdrantClient, models
import psycopg2
from psycopg2 import sql

# Load environment variables from .env file
load_dotenv()

# --- Configuration from Environment Variables ---
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
OPENAI_BASE_URL = os.getenv("OPENAI_BASE_URL")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
NEON_DATABASE_URL = os.getenv("NEON_DATABASE_URL")
FRONTEND_URL = os.getenv("FRONTEND_URL")

# Retry configuration
MAX_RETRIES = 3
RETRY_DELAY = 1  # exponential backoff multiplier

# --- FastAPI App Initialization ---
app = FastAPI(
    title="RAG Chatbot Backend API",
    description="API for interacting with a RAG chatbot for a Physical AI textbook."
)

# --- CORS Middleware ---
origins = [
    FRONTEND_URL if FRONTEND_URL else "http://localhost:3000",
    "http://localhost:3000",
    "http://localhost:3001",  # Added for dev server on port 3001
    "http://127.0.0.1:3000",
    "http://127.0.0.1:3001",
    "http://localhost",  # For local development
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],  # Allow all methods including OPTIONS
    allow_headers=["*"],  # Allow all headers
)

# --- Logging Configuration ---
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# --- OpenAI Client for Gemini API ---
openai_client = OpenAI(
    api_key=GEMINI_API_KEY,
    base_url=OPENAI_BASE_URL
)

# --- Qdrant Client ---
qdrant_client = QdrantClient(
    url=QDRANT_URL,
    api_key=QDRANT_API_KEY,
)
COLLECTION_NAME = "physical_ai_book"
VECTOR_SIZE = 768

# --- Postgres Connection (optional for chat history) ---
def get_db_connection():
    """Get database connection. Returns None if database is unavailable."""
    try:
        if not NEON_DATABASE_URL:
            logger.warning("NEON_DATABASE_URL not configured, skipping database")
            return None
        conn = psycopg2.connect(NEON_DATABASE_URL)
        return conn
    except Exception as e:
        logger.warning(f"Database connection failed (optional): {e}")
        return None

# Ensure chat_history table exists
def create_chat_history_table():
    """Create chat_history table if database is available."""
    conn = None
    try:
        conn = get_db_connection()
        if not conn:
            logger.info("Skipping chat_history table creation - database unavailable")
            return
        cur = conn.cursor()
        cur.execute("""
            CREATE TABLE IF NOT EXISTS chat_history (
                id SERIAL PRIMARY KEY,
                timestamp TIMESTAMP DEFAULT NOW(),
                question TEXT NOT NULL,
                answer TEXT NOT NULL,
                sources TEXT[],
                session_id VARCHAR(255)
            );
        """)
        conn.commit()
        cur.close()
        logger.info("chat_history table ensured to exist.")
    except Exception as e:
        logger.warning(f"Error creating chat_history table (optional): {e}")
    finally:
        if conn:
            conn.close()

@app.on_event("startup")
async def startup_event():
    create_chat_history_table()

# --- Pydantic Models ---
class ChatRequest(BaseModel):
    question: str = Field(..., min_length=1, max_length=2000, description="The question to ask about the book content")

class ChatSelectedRequest(BaseModel):
    question: str = Field(..., min_length=1, max_length=2000, description="The question to answer")
    selected_text: str = Field(..., min_length=1, max_length=5000, description="The selected text from the book")

class ChatResponse(BaseModel):
    answer: str = Field(..., description="The answer to the question")
    sources: List[str] = Field(default_factory=list, description="List of source file paths")

class HealthResponse(BaseModel):
    status: str = Field(..., description="Overall health status: ok or degraded")
    qdrant: str = Field(..., description="Qdrant connection status: connected or disconnected")
    postgres: str = Field(..., description="Postgres connection status: connected or disconnected")

# --- Helper Functions ---
def get_gemini_embedding(text: str) -> List[float]:
    """Generate embedding for text with retry logic and rate limit handling."""
    for attempt in range(MAX_RETRIES):
        try:
            response = openai_client.embeddings.create(
                model="text-embedding-004",
                input=text
            )
            logger.info(f"Successfully generated embedding for question")
            return response.data[0].embedding
        except RateLimitError as e:
            if attempt < MAX_RETRIES - 1:
                wait_time = RETRY_DELAY ** (attempt + 1)
                logger.warning(f"Rate limit hit. Retrying in {wait_time}s... (Attempt {attempt + 1}/{MAX_RETRIES})")
                time.sleep(wait_time)
            else:
                logger.error(f"Max retries exceeded for embedding: {e}")
                raise HTTPException(status_code=503, detail="Service unavailable: Rate limit exceeded")
        except Exception as e:
            logger.error(f"Error generating embedding: {e}")
            raise HTTPException(status_code=500, detail="Embedding generation failed")
    
    raise HTTPException(status_code=500, detail="Embedding generation failed")

def query_qdrant(embedding: List[float]) -> List[Dict[str, Any]]:
    """Query Qdrant for similar documents using embedding vector."""
    try:
        # FIXED: Reduced from 3 to 2 chunks
        search_result = qdrant_client.query_points(
            collection_name=COLLECTION_NAME,
            query=embedding,
            limit=2,  # REDUCED FROM 3 TO 2
            with_payload=True,
        )
        
        # Extract content and file paths from results
        results = []
        for hit in search_result.points:
            results.append({
                "content": hit.payload.get("content", ""),
                "file_path": hit.payload.get("file_path", "")
            })
        
        logger.info(f"Found {len(results)} relevant chunks from Qdrant")
        return results
        
    except Exception as e:
        logger.error(f"Error querying Qdrant: {e}")
        raise HTTPException(status_code=500, detail="Vector database query failed")

def generate_gemini_response(prompt: str) -> str:
    """Generate response from Gemini with retry logic and rate limit handling."""
    for attempt in range(MAX_RETRIES):
        try:
            response = openai_client.chat.completions.create(
                model="gemini-2.5-flash",  # Using stable model
                messages=[{"role": "user", "content": prompt}],
                temperature=0.7,
                max_tokens=2000  # REDUCED FROM 1000 TO 500 for faster responses
            )
            logger.info(f"Successfully generated Gemini response")
            
            # Validate response structure
            if not response.choices or len(response.choices) == 0:
                logger.warning(f"Gemini returned empty choices array, attempt {attempt + 1}/{MAX_RETRIES}")
                if attempt < MAX_RETRIES - 1:
                    wait_time = RETRY_DELAY ** (attempt + 1)
                    time.sleep(wait_time)
                    continue
                else:
                    raise HTTPException(status_code=500, detail="Gemini returned empty response")
            
            choice = response.choices[0]
            message = choice.message
            finish_reason = getattr(choice, 'finish_reason', None)
            
            # Try multiple ways to extract content
            content = None
            if hasattr(message, 'content'):
                content = message.content
            elif hasattr(message, 'text'):  # Some APIs use 'text' instead of 'content'
                content = message.text
            elif hasattr(choice, 'text'):  # Fallback to choice level
                content = choice.text
            
            # Handle case where content might be None or empty - retry if we have attempts left
            if content is None or (isinstance(content, str) and len(content.strip()) == 0):
                # Log detailed information for debugging
                logger.warning(
                    f"Gemini returned None/empty content, attempt {attempt + 1}/{MAX_RETRIES}. "
                    f"Finish reason: {finish_reason}, "
                    f"Message type: {type(message)}, "
                    f"Message attributes: {dir(message)}, "
                    f"Message dict: {message.model_dump() if hasattr(message, 'model_dump') else message.__dict__ if hasattr(message, '__dict__') else str(message)}"
                )
                
                # Check if there's a finish_reason that explains the issue
                if finish_reason == "content_filter":
                    logger.error("Content was filtered by safety settings")
                    raise HTTPException(status_code=400, detail="Response was filtered by content safety settings. Please rephrase your question.")
                elif finish_reason == "length":
                    logger.warning("Response was truncated due to length limit")
                    # Even if truncated, we might have partial content - check again
                    if content and len(content.strip()) > 0:
                        logger.info(f"Using truncated content: {len(content)} chars")
                        return content
                    elif attempt < MAX_RETRIES - 1:
                        wait_time = RETRY_DELAY ** (attempt + 1)
                        logger.info(f"Retrying in {wait_time}s...")
                        time.sleep(wait_time)
                        continue
                    else:
                        raise HTTPException(status_code=500, detail="Response was truncated and no content available")
                elif finish_reason == "stop":
                    # 'stop' usually means normal completion, but content is None - this is unusual
                    logger.warning("Finish reason is 'stop' but content is None - possible API issue")
                
                if attempt < MAX_RETRIES - 1:
                    wait_time = RETRY_DELAY ** (attempt + 1)
                    logger.info(f"Retrying in {wait_time}s...")
                    time.sleep(wait_time)
                    continue
                else:
                    # Last attempt failed, log full response structure for debugging
                    try:
                        response_dict = response.model_dump() if hasattr(response, 'model_dump') else response.__dict__ if hasattr(response, '__dict__') else str(response)
                    except Exception as e:
                        response_dict = f"Could not serialize response: {e}"
                    
                    logger.error(
                        f"Gemini returned None after all retries. "
                        f"Response structure: choices={len(response.choices) if response.choices else 0}, "
                        f"finish_reason={finish_reason}, "
                        f"full_response={response_dict}"
                    )
                    raise HTTPException(status_code=500, detail="AI model returned empty response. Please try again.")
            
            # Return successful content
            return content
            
        except HTTPException:
            # Re-raise HTTP exceptions (they're already properly formatted)
            raise
        except RateLimitError as e:
            if attempt < MAX_RETRIES - 1:
                wait_time = RETRY_DELAY ** (attempt + 1)
                logger.warning(f"Rate limit hit. Retrying in {wait_time}s... (Attempt {attempt + 1}/{MAX_RETRIES})")
                time.sleep(wait_time)
            else:
                logger.error(f"Max retries exceeded for LLM response: {e}")
                raise HTTPException(status_code=503, detail="Service unavailable: Rate limit exceeded")
        except Exception as e:
            logger.error(f"Error generating Gemini response: {e}")
            if attempt < MAX_RETRIES - 1:
                wait_time = RETRY_DELAY ** (attempt + 1)
                logger.info(f"Retrying in {wait_time}s... (Attempt {attempt + 1}/{MAX_RETRIES})")
                time.sleep(wait_time)
            else:
                raise HTTPException(status_code=500, detail="LLM response generation failed")
    
    raise HTTPException(status_code=500, detail="LLM response generation failed after all retries")

def save_chat_history(question: str, answer: str, sources: List[str], session_id: str = None):
    """Save chat history to database if available. Non-blocking if database unavailable."""
    conn = None
    try:
        conn = get_db_connection()
        if not conn:
            logger.debug("Skipping chat history save - database unavailable")
            return
        cur = conn.cursor()
        cur.execute(
            sql.SQL("INSERT INTO chat_history (question, answer, sources, session_id) VALUES (%s, %s, %s, %s)"),
            [question, answer, sources, session_id]
        )
        conn.commit()
        cur.close()
        logger.info("Chat history saved successfully.")
    except Exception as e:
        logger.warning(f"Error saving chat history (non-blocking): {e}")
        # Don't raise exception - chat should still respond even if history fails
    finally:
        if conn:
            conn.close()

# --- API Endpoints ---
@app.post("/api/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    """Ask a question about the book content and receive an answer with sources."""
    logger.info(f"POST /api/chat received: {request.question}")
    
    try:
        # Generate embedding for the question
        question_embedding = get_gemini_embedding(request.question)

        if not question_embedding:
            raise HTTPException(status_code=500, detail="Could not generate embedding for question.")

        # Query Qdrant for top 2 similar chunks (reduced from 3)
        relevant_chunks = query_qdrant(question_embedding)

        if not relevant_chunks:
            logger.warning("No relevant chunks found in Qdrant")
            context_text = "No relevant content found in the textbook."
            sources = []
        else:
            # FIXED: Shorten each chunk to 400 characters to reduce prompt length
            shortened_chunks = [chunk["content"][:400] + "..." if len(chunk["content"]) > 400 else chunk["content"] for chunk in relevant_chunks]
            context_text = "\n\n".join(shortened_chunks)
            sources = sorted(list(set([chunk["file_path"] for chunk in relevant_chunks])))
            
            logger.info(f"Context length: {len(context_text)} characters")

        # Construct prompt
        prompt = f"""Based on this context from a Physical AI textbook:

{context_text}

Answer this question concisely: {request.question}"""

        # Send to Gemini
        answer = generate_gemini_response(prompt)

        # Validate answer is not None
        if not answer:
            logger.error("Gemini returned None or empty response")
            raise HTTPException(status_code=500, detail="Failed to generate response from AI model")

        # Ensure sources is a list (should always be, but be defensive)
        if sources is None:
            sources = []

        # Save interaction to Neon Postgres
        save_chat_history(request.question, answer, sources)

        logger.info(f"POST /api/chat response: {len(answer)} chars, {len(sources)} sources")
        return ChatResponse(answer=answer, sources=sources)
    
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Unexpected error in /api/chat: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")

@app.post("/api/chat-selected", response_model=ChatResponse)
async def chat_selected(request: ChatSelectedRequest):
    """Explain selected text from the book."""
    logger.info(f"POST /api/chat-selected received: {request.question}, selected_text: {request.selected_text[:50]}...")
    
    try:
        # Use provided selected_text as context instead of Qdrant search
        # FIXED: Limit selected text to 1000 chars to prevent long prompts
        context_text = request.selected_text[:1000] + "..." if len(request.selected_text) > 1000 else request.selected_text

        # Construct prompt
        prompt = f"""Based on this selected text:

{context_text}

Answer the following concisely: {request.question}"""

        # Send to Gemini
        answer = generate_gemini_response(prompt)

        # Validate answer is not None
        if not answer:
            logger.error("Gemini returned None or empty response")
            raise HTTPException(status_code=500, detail="Failed to generate response from AI model")

        # For chat-selected, sources are empty since we're not using Qdrant
        sources = []

        # Save interaction to Neon Postgres
        save_chat_history(request.question, answer, sources)

        logger.info(f"POST /api/chat-selected response: {len(answer)} chars")
        return ChatResponse(answer=answer, sources=sources)
    
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Unexpected error in /api/chat-selected: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")

@app.get("/api/health", response_model=HealthResponse)
async def health_check():
    """Check the health status of the API and its dependencies."""
    logger.info("GET /api/health received")
    qdrant_status = "disconnected"
    postgres_status = "disconnected"

    try:
        # Check Qdrant connection
        qdrant_client.get_collection(collection_name=COLLECTION_NAME)
        qdrant_status = "connected"
        logger.info("Qdrant health check passed")
    except Exception as e:
        logger.error(f"Qdrant health check failed: {e}")

    conn = None
    try:
        # Check Postgres connection
        conn = get_db_connection()
        if conn:
            cur = conn.cursor()
            cur.execute("SELECT 1")
            cur.close()
            postgres_status = "connected"
            logger.info("Postgres health check passed")
    except Exception as e:
        logger.error(f"Postgres health check failed: {e}")
    finally:
        if conn:
            conn.close()

    status = "ok" if qdrant_status == "connected" and postgres_status == "connected" else "degraded"
    logger.info(f"GET /api/health response: status={status}, qdrant={qdrant_status}, postgres={postgres_status}")
    return HealthResponse(status=status, qdrant=qdrant_status, postgres=postgres_status)