import os
import re
import time
import logging
from dotenv import load_dotenv
from openai import OpenAI, RateLimitError
from qdrant_client import QdrantClient, models
import traceback

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Load environment variables
load_dotenv()

GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
OPENAI_BASE_URL = os.getenv("OPENAI_BASE_URL")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

# Initialize OpenAI client for Gemini API
client = OpenAI(
    api_key=GEMINI_API_KEY,
    base_url=OPENAI_BASE_URL
)

# Initialize Qdrant client
qdrant_client = QdrantClient(
    url=QDRANT_URL,
    api_key=QDRANT_API_KEY,
)

COLLECTION_NAME = "physical_ai_book"
VECTOR_SIZE = 768  # Dimension for text-embedding-004
CHUNK_SIZE = 1000  # 800-1200 range, using 1000 as average
CHUNK_OVERLAP = 100
MAX_RETRIES = 3
RETRY_DELAY = 2  # exponential backoff
BATCH_SIZE = 50  # Upload to Qdrant in batches

def create_qdrant_collection():
    """Create or recreate Qdrant collection."""
    logger.info(f"Creating Qdrant collection: {COLLECTION_NAME}")
    try:
        # Check if collection exists
        collections = qdrant_client.get_collections().collections
        collection_names = [col.name for col in collections]
        
        if COLLECTION_NAME in collection_names:
            logger.info(f"Collection {COLLECTION_NAME} exists. Deleting...")
            qdrant_client.delete_collection(collection_name=COLLECTION_NAME)
        
        # Create new collection
        qdrant_client.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=models.VectorParams(size=VECTOR_SIZE, distance=models.Distance.COSINE),
        )
        logger.info(f"Collection {COLLECTION_NAME} created successfully.")
    except Exception as e:
        logger.error(f"Error creating Qdrant collection: {e}")
        logger.error(traceback.format_exc())
        raise

def get_embedding(text: str) -> list:
    """Generate embedding for text with retry logic and rate limit handling."""
    if not text or not text.strip():
        logger.warning("Empty text provided for embedding")
        return None
        
    for attempt in range(MAX_RETRIES):
        try:
            logger.debug(f"Generating embedding for text (first 50 chars): {text[:50]}...")
            response = client.embeddings.create(
                model="text-embedding-004",
                input=text
            )
            return response.data[0].embedding
        except RateLimitError as e:
            if attempt < MAX_RETRIES - 1:
                wait_time = RETRY_DELAY ** (attempt + 1)
                logger.warning(f"Rate limit hit. Retrying in {wait_time}s... (Attempt {attempt + 1}/{MAX_RETRIES})")
                time.sleep(wait_time)
            else:
                logger.error(f"Max retries exceeded for embedding generation: {e}")
                raise
        except Exception as e:
            logger.error(f"Error generating embedding: {e}")
            logger.error(traceback.format_exc())
            return None
    return None

def chunk_text(content: str, file_path: str, module_name: str) -> list:
    """
    Chunk text into 800-1200 character segments with 100 character overlap.
    Strip frontmatter before processing.
    """
    try:
        # Strip frontmatter (--- sections at top)
        content_without_frontmatter = re.sub(r'^---.*?---\s*', '', content, flags=re.DOTALL, count=1)
        
        if not content_without_frontmatter.strip():
            logger.warning(f"No content after stripping frontmatter from {file_path}")
            return []
        
        content_length = len(content_without_frontmatter)
        logger.info(f"  Content length: {content_length} characters")
        
        chunks = []
        current_pos = 0
        chunk_index = 0
        iteration_count = 0
        max_iterations = (content_length // (CHUNK_SIZE - CHUNK_OVERLAP)) + 10  # Safety limit
        
        while current_pos < content_length:
            iteration_count += 1
            
            # Safety check: prevent infinite loops
            if iteration_count > max_iterations:
                logger.error(f"Exceeded max iterations ({max_iterations}) for {file_path}. Breaking.")
                break
            
            # Extract chunk of CHUNK_SIZE characters
            chunk_end = min(current_pos + CHUNK_SIZE, content_length)
            chunk_content = content_without_frontmatter[current_pos:chunk_end]
            
            # If we're not at the end, try to break at a word boundary
            if chunk_end < content_length:
                # Look for the last newline or space within a reasonable range
                last_newline = chunk_content.rfind('\n')
                last_space = chunk_content.rfind(' ')
                break_point = max(last_newline, last_space)
                
                if break_point > CHUNK_SIZE * 0.8:  # Only break if it's reasonably close to our target
                    chunk_content = chunk_content[:break_point + 1]
                    chunk_end = current_pos + len(chunk_content)
            
            # Only add non-empty chunks
            if chunk_content.strip():
                chunk_dict = {
                    "content": chunk_content.strip(),
                    "file_path": file_path,
                    "module": module_name,
                    "chunk_index": chunk_index
                }
                chunks.append(chunk_dict)
                chunk_index += 1
            
            # Move to next chunk position with overlap
            next_pos = chunk_end - CHUNK_OVERLAP
            
            # CRITICAL: Ensure we always move forward
            if next_pos <= current_pos:
                next_pos = chunk_end  # Force forward movement
            
            current_pos = next_pos
            
            # Log progress for large files
            if chunk_index % 50 == 0 and chunk_index > 0:
                logger.info(f"  Progress: {chunk_index} chunks created, position {current_pos}/{content_length}")
        
        logger.info(f"  Chunking complete: {len(chunks)} chunks created")
        return chunks
        
    except Exception as e:
        logger.error(f"Error chunking text from {file_path}: {e}")
        logger.error(traceback.format_exc())
        return []

def embed_book_content():
    """Process all markdown files and embed them into Qdrant."""
    docs_path = os.path.join(os.path.dirname(__file__), "..", "docs")
    
    if not os.path.exists(docs_path):
        logger.error(f"Documentation directory not found: {docs_path}")
        raise FileNotFoundError(f"Documentation directory not found: {docs_path}")
    
    logger.info(f"Starting book embedding from {docs_path}")
    create_qdrant_collection()

    all_chunks = []
    file_count = 0
    
    for root, _, files in os.walk(docs_path):
        for file in files:
            if file.endswith(".md"):
                file_path_abs = os.path.join(root, file)
                # Relative path from docs/
                file_path_rel = os.path.relpath(file_path_abs, docs_path).replace("\\", "/")
                
                # Extract module name - FIX: Handle files in root docs/ folder
                parent_dir = os.path.dirname(file_path_rel)
                if parent_dir == "" or parent_dir == ".":
                    module_name = "general"
                else:
                    # Get first directory in path (e.g., "foundations" from "foundations/file.md")
                    module_name = parent_dir.split("/")[0]

                logger.info(f"Processing file: {file_path_rel}, Module: {module_name}")

                try:
                    with open(file_path_abs, "r", encoding="utf-8") as f:
                        content = f.read()
                    
                    # Log file size
                    file_size = len(content)
                    logger.info(f"  File size: {file_size} characters")

                    file_chunks = chunk_text(content, file_path_rel, module_name)
                    all_chunks.extend(file_chunks)
                    file_count += 1
                    logger.info(f"  - Extracted {len(file_chunks)} chunks from {file_path_rel}")
                except Exception as e:
                    logger.error(f"Error processing file {file_path_rel}: {e}")
                    logger.error(traceback.format_exc())
                    continue

    logger.info(f"Total files processed: {file_count}")
    logger.info(f"Total chunks created: {len(all_chunks)}")

    if not all_chunks:
        logger.warning("No chunks created. Nothing to embed.")
        return

    # Process embeddings and upload in batches
    embedded_count = 0
    failed_count = 0
    points_batch = []
    
    for i, chunk_data in enumerate(all_chunks):
        try:
            embedding = get_embedding(chunk_data["content"])
            if embedding:
                points_batch.append(
                    models.PointStruct(
                        id=i,
                        vector=embedding,
                        payload=chunk_data
                    )
                )
                embedded_count += 1
                
                # Upload in batches
                if len(points_batch) >= BATCH_SIZE:
                    logger.info(f"Uploading batch of {len(points_batch)} points to Qdrant...")
                    try:
                        qdrant_client.upsert(
                            collection_name=COLLECTION_NAME,
                            wait=True,
                            points=points_batch
                        )
                        logger.info(f"Progress: {embedded_count}/{len(all_chunks)} embeddings completed")
                        points_batch = []
                    except Exception as e:
                        logger.error(f"Error upserting batch to Qdrant: {e}")
                        logger.error(traceback.format_exc())
                        raise
                
                # Rate limiting: small delay between embeddings
                if embedded_count % 10 == 0:
                    time.sleep(0.5)
            else:
                failed_count += 1
                logger.warning(f"Failed to generate embedding for chunk {i} from {chunk_data['file_path']}")
        except Exception as e:
            failed_count += 1
            logger.error(f"Error processing chunk {i}: {e}")
            logger.error(traceback.format_exc())
            continue

    # Upload remaining points
    if points_batch:
        logger.info(f"Uploading final batch of {len(points_batch)} points to Qdrant...")
        try:
            qdrant_client.upsert(
                collection_name=COLLECTION_NAME,
                wait=True,
                points=points_batch
            )
        except Exception as e:
            logger.error(f"Error upserting final batch to Qdrant: {e}")
            logger.error(traceback.format_exc())
            raise

    logger.info(f"Embedding complete. Successfully embedded {embedded_count} chunks.")
    if failed_count > 0:
        logger.warning(f"Failed to embed {failed_count} chunks.")

if __name__ == "__main__":
    try:
        embed_book_content()
    except Exception as e:
        logger.error(f"Fatal error during book embedding: {e}")
        logger.error(traceback.format_exc())
        raise