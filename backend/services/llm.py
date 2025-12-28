import os
import openai
import time
from typing import List, Optional, Dict, Any
from dotenv import load_dotenv
from utils.logger import logger

# Load environment variables
load_dotenv()

class LLMService:
    """
    Service class for interacting with the OpenAI SDK configured for Gemini API.
    """

    def __init__(self):
        # Configure OpenAI client for Gemini API
        self.api_key = os.getenv("GEMINI_API_KEY")
        if not self.api_key:
            raise ValueError("GEMINI_API_KEY environment variable is required")

        self.base_url = os.getenv("OPENAI_BASE_URL", "https://generativelanguage.googleapis.com/v1beta/openai/")

        # Initialize OpenAI client with Gemini configuration
        self.client = openai.OpenAI(
            api_key=self.api_key,
            base_url=self.base_url
        )

        # Set default model names
        self.embedding_model = "text-embedding-004"  # For embeddings
        self.gemini_model = "gemini-2.5-flash"  # For chat completions

    def generate_embedding(self, text: str, max_retries: int = 3) -> List[float]:
        """
        Generate embedding for the given text using the embedding model.

        Args:
            text: Text to generate embedding for
            max_retries: Maximum number of retry attempts

        Returns:
            List of floats representing the embedding vector
        """
        for attempt in range(max_retries):
            try:
                response = self.client.embeddings.create(
                    input=text,
                    model=self.embedding_model
                )

                embedding = response.data[0].embedding
                logger.info(f"Generated embedding for text of length {len(text)}")
                return embedding
            except Exception as e:
                logger.error(f"Attempt {attempt + 1} failed to generate embedding: {str(e)}")
                if attempt == max_retries - 1:
                    raise
                # Exponential backoff: wait 2^attempt seconds
                time.sleep(2 ** attempt)

    def generate_embeddings_batch(self, texts: List[str], max_retries: int = 3) -> List[List[float]]:
        """
        Generate embeddings for a batch of texts.

        Args:
            texts: List of texts to generate embeddings for
            max_retries: Maximum number of retry attempts

        Returns:
            List of embedding vectors
        """
        embeddings = []
        for text in texts:
            embedding = self.generate_embedding(text, max_retries)
            embeddings.append(embedding)
        return embeddings

    def generate_response(self, prompt: str, max_retries: int = 3, temperature: float = 0.7) -> str:
        """
        Generate a response from the LLM based on the given prompt.

        Args:
            prompt: Input prompt for the LLM
            max_retries: Maximum number of retry attempts
            temperature: Controls randomness of the response

        Returns:
            Generated response from the LLM
        """
        for attempt in range(max_retries):
            try:
                response = self.client.chat.completions.create(
                    model=self.gemini_model,
                    messages=[{"role": "user", "content": prompt}],
                    temperature=temperature
                )

                answer = response.choices[0].message.content
                logger.info(f"Generated response for prompt of length {len(prompt)}")
                return answer
            except Exception as e:
                logger.error(f"Attempt {attempt + 1} failed to generate response: {str(e)}")
                if attempt == max_retries - 1:
                    raise
                # Exponential backoff: wait 2^attempt seconds
                time.sleep(2 ** attempt)

    def generate_response_with_context(self, question: str, context: str, max_retries: int = 3) -> str:
        """
        Generate a response based on a question and provided context.

        Args:
            question: The user's question
            context: Context to use for answering the question
            max_retries: Maximum number of retry attempts

        Returns:
            Generated response from the LLM
        """
        prompt = f"""
        Context: {context}

        Question: {question}

        Please provide a comprehensive answer to the question based on the provided context.
        If the context doesn't contain information to answer the question, please say so.
        """

        return self.generate_response(prompt, max_retries)

# Create a singleton instance
llm_service = LLMService()

def get_llm_service() -> LLMService:
    """Get the singleton LLM service instance."""
    return llm_service