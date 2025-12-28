import os
import glob
import re
import markdown
import uuid
from typing import List, Dict, Any
from dotenv import load_dotenv
from models.content import ContentSegment
from services.qdrant import qdrant_service
from services.llm import llm_service
from utils.logger import logger

# Load environment variables
load_dotenv()

def strip_frontmatter(content: str) -> str:
    """
    Remove frontmatter from markdown content if present.

    Args:
        content: Raw markdown content

    Returns:
        Content with frontmatter removed
    """
    # Look for frontmatter pattern (--- to ---)
    frontmatter_pattern = r'^---\s*\n.*?\n---\s*\n'
    content_without_frontmatter = re.sub(frontmatter_pattern, '', content, flags=re.DOTALL)
    return content_without_frontmatter.strip()


def segment_content(content: str, max_length: int = 1000, overlap: int = 100) -> List[str]:
    """
    Segment content into chunks of specified length with overlap.

    Args:
        content: Content to segment
        max_length: Maximum length of each segment (800-1200 chars as per spec)
        overlap: Number of characters to overlap between segments

    Returns:
        List of content segments
    """
    segments = []
    start = 0

    while start < len(content):
        end = start + max_length

        # If we're near the end, include the rest
        if end >= len(content):
            segment = content[start:]
            if segment.strip():
                segments.append(segment)
            break

        # Find a good breaking point (try to break at sentence or paragraph)
        break_point = end
        for i in range(end, start, -1):
            if content[i] in '.!?\n\t ' and i - start > max_length // 2:
                break_point = i + 1
                break

        segment = content[start:break_point]
        segments.append(segment)

        start = break_point - overlap

    return segments


def get_source_identifier(file_path: str) -> str:
    """
    Generate a source identifier from the file path.

    Args:
        file_path: Path to the source file

    Returns:
        Relative path from docs directory
    """
    # Get the relative path from the docs directory
    abs_file_path = os.path.abspath(file_path)
    docs_dir = os.path.abspath('../docs/')

    try:
        rel_path = os.path.relpath(abs_file_path, docs_dir)
        return rel_path.replace('\\', '/')  # Normalize path separators
    except ValueError:
        # If docs_dir is not a parent of file_path, return the basename
        return os.path.basename(file_path)


def get_chapter_or_module(file_path: str) -> str:
    """
    Extract chapter or module from the file path.

    Args:
        file_path: Path to the source file

    Returns:
        Chapter or module identifier
    """
    abs_file_path = os.path.abspath(file_path)
    docs_dir = os.path.abspath('../docs/')

    try:
        rel_path = os.path.relpath(abs_file_path, docs_dir)
        # Get the first directory in the path
        parts = rel_path.split(os.sep)
        if len(parts) > 1 and parts[0] != '..':
            return parts[0]
        else:
            return 'root'
    except ValueError:
        return 'unknown'


def generate_point_id(source_id: str, idx: int) -> int:
    """
    Generate a numeric ID from source_id and index.
    Qdrant requires numeric IDs for points.
    
    Args:
        source_id: Source identifier string
        idx: Segment index
        
    Returns:
        Numeric ID
    """
    # Create a unique string and hash it
    unique_str = f"{source_id}_{idx}"
    # Use hash to generate a positive integer
    return abs(hash(unique_str)) % (10 ** 10)


def embed_book():
    """
    Main function to embed book content into Qdrant.
    """
    logger.info("Starting book embedding process...")

    # Check Qdrant connection first
    if not qdrant_service.check_connection():
        logger.error("Failed to connect to Qdrant. Please ensure Qdrant is running on http://localhost:6333")
        logger.info("Start Qdrant with: docker run -p 6333:6333 qdrant/qdrant")
        return

    # Get vector size from the embedding model
    # text-embedding-004 produces 768-dimensional vectors
    vector_size = 768
    
    # Create the physical_ai_book collection in Qdrant
    logger.info(f"Creating collection 'physical_ai_book' with vector size {vector_size}")
    qdrant_service.create_collection("physical_ai_book", vector_size=vector_size)

    # Find all markdown files in the docs directory
    docs_path = "../docs/**/*.md"
    md_files = glob.glob(docs_path, recursive=True)

    # Also look for .mdx files which are also markdown
    docs_path_mdx = "../docs/**/*.mdx"
    mdx_files = glob.glob(docs_path_mdx, recursive=True)

    all_files = md_files + mdx_files
    logger.info(f"Found {len(all_files)} markdown files to process")

    if not all_files:
        logger.warning("No markdown files found in ../docs/. Please ensure the docs directory exists with markdown files.")
        logger.info("Current working directory: " + os.getcwd())
        logger.info("Looking for files in: " + os.path.abspath("../docs/"))
        return

    total_segments = 0

    # Process each file
    for i, file_path in enumerate(all_files):
        logger.info(f"Processing file {i+1}/{len(all_files)}: {file_path}")

        try:
            # Read the file content
            with open(file_path, 'r', encoding='utf-8') as f:
                raw_content = f.read()

            # Strip frontmatter
            content = strip_frontmatter(raw_content)

            # Skip empty files
            if not content.strip():
                logger.warning(f"File {file_path} is empty after removing frontmatter")
                continue

            # Segment the content
            segments = segment_content(content)
            logger.info(f"Created {len(segments)} segments from {file_path}")

            # Process each segment
            points = []
            for idx, segment in enumerate(segments):
                if segment.strip():  # Skip empty segments
                    # Generate embedding for the segment
                    logger.info(f"  Generating embedding for segment {idx+1}/{len(segments)}")
                    embedding = llm_service.generate_embedding(segment)

                    # Create source identifier and chapter/module info
                    source_id = get_source_identifier(file_path)
                    chapter_module = get_chapter_or_module(file_path)

                    # Generate numeric ID for the point
                    point_id = generate_point_id(source_id, idx)

                    # Create Qdrant point
                    point = {
                        "id": point_id,
                        "vector": embedding,
                        "payload": {
                            "text": segment,
                            "source_identifier": source_id,
                            "chapter_or_module": chapter_module,
                            "segment_index": idx
                        }
                    }
                    points.append(point)

            # Upsert all points for this file to Qdrant
            if points:
                qdrant_service.upsert_vectors("physical_ai_book", points)
                total_segments += len(points)
                logger.info(f"✅ Embedded {len(points)} segments from {file_path}")
            else:
                logger.warning(f"No segments created for {file_path}")

        except Exception as e:
            logger.error(f"❌ Error processing file {file_path}: {str(e)}")
            import traceback
            logger.error(traceback.format_exc())
            continue  # Continue with next file even if one fails

    logger.info("=" * 60)
    logger.info(f"✅ Book embedding process completed!")
    logger.info(f"Total files processed: {len(all_files)}")
    logger.info(f"Total segments embedded: {total_segments}")
    logger.info("=" * 60)


if __name__ == "__main__":
    embed_book()