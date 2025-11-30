/**
 * REST API client for Chat Widget
 * Handles all API calls to the backend with retry logic and error handling
 * @module services/api
 */

import { ChatRequest, ChatSelectedRequest, ChatResponse } from '../types/chat';
import { getBackendUrl } from '../config/api';

/** Default timeout for API requests in milliseconds */
const API_TIMEOUT_MS = 60000; // Increased to 60s to accommodate Gemini API response times

/** Maximum number of retry attempts */
const MAX_RETRIES = 3;

/** Base delay for exponential backoff in milliseconds */
const INITIAL_BACKOFF_MS = 1000;

/**
 * Generates a unique request ID for idempotency and tracking
 * @returns {string} Unique request ID
 */
function generateRequestId(): string {
  return `req-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
}

/**
 * Formats error messages for user display
 * @param {number} status - HTTP status code
 * @param {string} message - Error message
 * @returns {string} User-friendly error message
 */
function getUserFriendlyErrorMessage(status: number, message: string): string {
  if (status === 0 || message.includes('network')) {
    return 'Check your internet connection';
  }
  if (status === 503 || message.includes('rate limit')) {
    return 'AI service is busy. Please try again in a moment.';
  }
  if (status >= 500) {
    return 'Failed to connect to AI. Please try again.';
  }
  if (status === 408 || message.includes('timeout')) {
    return 'Request took too long. Please try again.';
  }
  if (status === 400) {
    return 'Invalid question format. Please check and try again.';
  }
  return 'Something went wrong. Please try again.';
}

/**
 * Executes an API call with retry logic and exponential backoff
 * @template T
 * @param {() => Promise<T>} fn - Function that makes the API call
 * @param {number} retryCount - Current retry attempt
 * @returns {Promise<T>} API response
 * @throws {APIError} If all retries exhausted
 */
async function withRetry<T>(
  fn: () => Promise<T>,
  retryCount = 0
): Promise<T> {
  try {
    return await fn();
  } catch (error) {
    if (retryCount < MAX_RETRIES && shouldRetry(error)) {
      const backoffDelay = INITIAL_BACKOFF_MS * Math.pow(2, retryCount);
      await new Promise((resolve) => setTimeout(resolve, backoffDelay));
      return withRetry(fn, retryCount + 1);
    }
    throw error;
  }
}

/**
 * Determines if an error should trigger a retry
 * @param {unknown} error - The error object
 * @returns {boolean} Whether to retry
 */
function shouldRetry(error: unknown): boolean {
  if (error instanceof APIError) {
    // Retry on network errors (status 0), timeouts (408), and server errors (503)
    return error.status === 0 || error.status === 408 || error.status === 503;
  }
  // Assume network-like errors should retry
  return true;
}

/**
 * Posts a question to the chat API
 * @param {string} question - User's question (1-2000 characters)
 * @returns {Promise<ChatResponse>} API response with answer and sources
 * @throws {APIError} If request fails
 */
export async function postChat(question: string): Promise<ChatResponse> {
  const requestId = generateRequestId();

  return withRetry(async () => {
    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), API_TIMEOUT_MS);

    try {
      const response = await fetch(`${getBackendUrl()}/api/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'X-Request-ID': requestId,
        },
        body: JSON.stringify({ question } as ChatRequest),
        signal: controller.signal,
      });

      if (!response.ok) {
        const message = `HTTP ${response.status}`;
        const userMessage = getUserFriendlyErrorMessage(response.status, message);
        throw new APIError(response.status, message, userMessage);
      }

      const data: ChatResponse = await response.json();

      // Validate response structure
      if (typeof data.answer !== 'string' || !Array.isArray(data.sources)) {
        throw new APIError(500, 'Invalid response format', 'Failed to parse response');
      }

      return data;
    } catch (error) {
      if (error instanceof APIError) {
        throw error;
      }

      if (error instanceof TypeError) {
        // Network error or CORS issue
        const userMessage = getUserFriendlyErrorMessage(0, error.message);
        throw new APIError(0, error.message, userMessage);
      }

      if ((error as DOMException).name === 'AbortError') {
        // Timeout
        throw new APIError(408, 'Request timeout', 'Request took too long. Please try again.');
      }

      // Unknown error
      throw new APIError(500, String(error), 'Something went wrong. Please try again.');
    } finally {
      clearTimeout(timeoutId);
    }
  });
}

/**
 * Posts a question with selected text context to the chat API
 * @param {string} question - User's question
 * @param {string} selectedText - Selected text from the page (max 5000 characters)
 * @returns {Promise<ChatResponse>} API response with explanation
 * @throws {APIError} If request fails
 */
export async function postChatSelected(
  question: string,
  selectedText: string
): Promise<ChatResponse> {
  const requestId = generateRequestId();

  return withRetry(async () => {
    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), API_TIMEOUT_MS);

    try {
      const response = await fetch(`${getBackendUrl()}/api/chat-selected`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'X-Request-ID': requestId,
        },
        body: JSON.stringify({ question, selected_text: selectedText } as ChatSelectedRequest),
        signal: controller.signal,
      });

      if (!response.ok) {
        const message = `HTTP ${response.status}`;
        const userMessage = getUserFriendlyErrorMessage(response.status, message);
        throw new APIError(response.status, message, userMessage);
      }

      const data: ChatResponse = await response.json();

      // Validate response structure
      if (typeof data.answer !== 'string' || !Array.isArray(data.sources)) {
        throw new APIError(500, 'Invalid response format', 'Failed to parse response');
      }

      return data;
    } catch (error) {
      if (error instanceof APIError) {
        throw error;
      }

      if (error instanceof TypeError) {
        // Network error or CORS issue
        const userMessage = getUserFriendlyErrorMessage(0, error.message);
        throw new APIError(0, error.message, userMessage);
      }

      if ((error as DOMException).name === 'AbortError') {
        // Timeout
        throw new APIError(408, 'Request timeout', 'Request took too long. Please try again.');
      }

      // Unknown error
      throw new APIError(500, String(error), 'Something went wrong. Please try again.');
    } finally {
      clearTimeout(timeoutId);
    }
  });
}

/**
 * APIError class for structured error handling
 */
export class APIError extends Error {
  public status: number;
  public userMessage: string;

  constructor(status: number, message: string, userMessage: string) {
    super(message);
    this.name = 'APIError';
    this.status = status;
    this.userMessage = userMessage;

    // Log errors for debugging
    if (typeof window !== 'undefined' && window.console) {
      console.error(`[ChatWidget API Error] ${this.name}: ${message} (${status})`, {
        requestId: undefined,
        timestamp: new Date().toISOString(),
        userMessage,
      });
    }
  }
}
