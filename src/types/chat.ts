/**
 * Chat Widget TypeScript Types and Interfaces
 * Complete type definitions for all chat widget entities and API structures
 * @module types/chat
 */

/**
 * Represents a single message in a conversation
 * @interface ChatMessage
 */
export interface ChatMessage {
  /** Unique identifier for the message (UUID or timestamp-based) */
  id: string;

  /** Role of the message sender */
  role: 'user' | 'assistant';

  /** Message content text (max 2000 chars for user, variable for assistant) */
  content: string;

  /** ISO 8601 timestamp when message was created */
  timestamp: string; // ISO 8601 format

  /** Array of source file paths (only for assistant messages) */
  sources: string[];

  /** Flag indicating if message failed to send */
  error?: boolean;
}

/**
 * Represents an entire chat session
 * @interface Conversation
 */
export interface Conversation {
  /** Session ID (optional, used for persistence) */
  id?: string;

  /** Array of all messages in conversation (ordered by timestamp) */
  messages: ChatMessage[];

  /** ISO 8601 timestamp when conversation started */
  createdAt: string;

  /** ISO 8601 timestamp when conversation was last updated */
  updatedAt: string;

  /** Whether the chat modal is currently open */
  isOpen: boolean;
}

/**
 * Complete state of the chat widget UI and data
 * @interface ChatWidgetState
 */
export interface ChatWidgetState {
  /** Current conversation and messages */
  conversation: Conversation;

  /** Loading state during API request */
  isLoading: boolean;

  /** Error message if a request failed (null if no error) */
  error: string | null;

  /** Current text in message input field */
  inputValue: string;

  /** Selected text from page (null if no selection) */
  selectedText: string | null;

  /** Whether in "explain selected text" mode */
  selectedTextMode: boolean;
}

/**
 * API request payload for /api/chat endpoint
 * @interface ChatRequest
 */
export interface ChatRequest {
  /** Question about the book content (1-2000 characters) */
  question: string;
}

/**
 * API request payload for /api/chat-selected endpoint
 * @interface ChatSelectedRequest
 */
export interface ChatSelectedRequest {
  /** Question about the selected text */
  question: string;

  /** Selected text content from page (max 5000 characters) */
  selected_text: string;
}

/**
 * API response payload from both chat endpoints
 * @interface ChatResponse
 */
export interface ChatResponse {
  /** AI-generated answer to the question */
  answer: string;

  /** Array of source file paths cited in the answer */
  sources: string[];
}

/**
 * Type for API request with retry metadata
 * @interface APIRequestMetadata
 */
export interface APIRequestMetadata {
  /** Unique request ID for idempotency */
  requestId: string;

  /** Timestamp when request was made */
  timestamp: string;

  /** Retry attempt number */
  retryCount: number;

  /** Maximum retries allowed */
  maxRetries: number;
}

/**
 * Union type for API request types
 */
export type APIRequest = ChatRequest | ChatSelectedRequest;
