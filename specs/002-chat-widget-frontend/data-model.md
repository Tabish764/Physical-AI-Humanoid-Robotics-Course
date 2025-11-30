# Data Model: Chat Widget Frontend

**Date**: 2025-11-29
**Feature**: Interactive AI Chat Widget for Docusaurus Book
**Status**: Phase 1 Design

## Entities

### 1. ChatMessage

Represents a single message in the conversation (user or assistant).

**Attributes**:
- `id`: string (unique identifier, UUID or timestamp-based)
- `role`: 'user' | 'assistant' (who sent the message)
- `content`: string (message text, max 2000 chars for user, variable for assistant)
- `timestamp`: Date (ISO 8601 format, e.g., "2025-11-29T14:30:00.000Z")
- `sources`: string[] (array of file paths, only for assistant messages, empty array for user)
- `error`: boolean (optional, true if message failed to send)

**Validation Rules**:
- `id`: Must be unique within a conversation
- `role`: Must be exactly 'user' or 'assistant'
- `content`: Min 1 char, max 2000 for user messages; variable for assistant (typically < 5000)
- `timestamp`: Valid ISO 8601 date
- `sources`: Array of non-empty strings (file paths), max 10 sources

**Example User Message**:
```typescript
{
  id: "msg-1701284400000",
  role: "user",
  content: "What is ROS 2?",
  timestamp: "2025-11-29T14:33:20Z",
  sources: [],
  error: false
}
```

**Example Assistant Message**:
```typescript
{
  id: "msg-1701284405000",
  role: "assistant",
  content: "ROS 2 (Robot Operating System 2) is a middleware for robotics...",
  timestamp: "2025-11-29T14:33:25Z",
  sources: ["module1-ros2/ros2-architecture.md", "module1-ros2/ros2-packages-python.md"],
  error: false
}
```

---

### 2. Conversation

Represents the entire chat session (collection of messages and metadata).

**Attributes**:
- `id`: string (session ID, optional for MVP)
- `messages`: ChatMessage[] (array of all messages in order)
- `createdAt`: Date (when conversation started)
- `updatedAt`: Date (when last message was added/modified)
- `isOpen`: boolean (whether modal is currently open)

**Validation Rules**:
- `messages`: Array must be ordered by timestamp (ascending)
- `createdAt`, `updatedAt`: Valid ISO 8601 dates, `updatedAt` >= `createdAt`
- `isOpen`: Boolean value only

**Example Conversation**:
```typescript
{
  id: "session-2025-11-29",
  messages: [
    { id: "msg-1", role: "user", content: "...", timestamp: "...", sources: [] },
    { id: "msg-2", role: "assistant", content: "...", timestamp: "...", sources: [...] },
    { id: "msg-3", role: "user", content: "...", timestamp: "...", sources: [] }
  ],
  createdAt: "2025-11-29T14:33:00Z",
  updatedAt: "2025-11-29T14:35:00Z",
  isOpen: true
}
```

---

### 3. ChatWidgetState

Represents the current state of the chat widget (UI state + conversation).

**Attributes**:
- `conversation`: Conversation (the chat history and metadata)
- `isLoading`: boolean (true while waiting for API response)
- `error`: string | null (error message if a request failed)
- `inputValue`: string (current text in input field)
- `selectedText`: string | null (selected text from page, if any)
- `selectedTextMode`: boolean (true if in "explain selected text" mode)

**Validation Rules**:
- `isLoading`: Boolean only
- `error`: Null or non-empty string (user-friendly message)
- `inputValue`: String, max 2000 chars
- `selectedText`: Null or non-empty string, max 5000 chars
- `selectedTextMode`: Boolean only
- At any time: If `selectedTextMode` is true, `selectedText` must be non-null

**Example Widget State**:
```typescript
{
  conversation: {
    id: "session-2025-11-29",
    messages: [...],
    createdAt: "...",
    updatedAt: "...",
    isOpen: true
  },
  isLoading: false,
  error: null,
  inputValue: "What is...",
  selectedText: null,
  selectedTextMode: false
}
```

**Example Widget State (Loading)**:
```typescript
{
  conversation: { ... },
  isLoading: true,
  error: null,
  inputValue: "",
  selectedText: null,
  selectedTextMode: false
}
```

**Example Widget State (Error)**:
```typescript
{
  conversation: { ... },
  isLoading: false,
  error: "Check your internet connection",
  inputValue: "What is ROS 2?",
  selectedText: null,
  selectedTextMode: false
}
```

---

### 4. APIRequest

Represents a request to the backend API.

**Chat Endpoint Request**:
- `question`: string (user's question, 1-2000 chars)

**Selected Text Endpoint Request**:
- `question`: string (user's question, 1-2000 chars)
- `selected_text`: string (selected text from page, 1-5000 chars)

**Example Requests**:
```typescript
// POST /api/chat
{
  question: "What is ROS 2?"
}

// POST /api/chat-selected
{
  question: "Explain this in simple terms",
  selected_text: "ROS 2 (Robot Operating System 2) is a middleware..."
}
```

---

### 5. APIResponse

Represents a response from the backend API.

**Attributes**:
- `answer`: string (AI-generated response)
- `sources`: string[] (array of file paths from the book)

**Validation Rules**:
- `answer`: Non-empty string (< 5000 chars typically)
- `sources`: Array of non-empty strings (file paths), max 10 sources

**Example Response**:
```typescript
{
  answer: "ROS 2 (Robot Operating System 2) is a middleware for robotics that...",
  sources: ["module1-ros2/ros2-architecture.md", "module1-ros2/ros2-packages-python.md"]
}
```

---

## State Transitions

### Conversation Flow

```
Initial State:
  conversation.messages = []
  isLoading = false
  error = null
  inputValue = ""

User Types:
  inputValue = "What is ROS 2?"

User Sends Message:
  → Add message to conversation.messages (role: "user", content: inputValue)
  → isLoading = true
  → inputValue = ""
  → Call API

API Response (Success):
  → Add message to conversation.messages (role: "assistant", content: response.answer, sources: response.sources)
  → isLoading = false
  → error = null
  → Update conversation.updatedAt

API Response (Error):
  → isLoading = false
  → error = "Error message (specific type)"
  → Show retry button

User Clicks Retry:
  → isLoading = true
  → error = null
  → Call API again (same last message)

User Closes Modal:
  → conversation.isOpen = false
  → conversation remains in memory (state not cleared)

User Opens Modal Again:
  → conversation.isOpen = true
  → conversation.messages still present (all previous messages visible)
```

---

## Relationships

**Conversation ↔ ChatMessage**: One-to-Many
- A conversation contains many messages
- Each message belongs to exactly one conversation

**ChatWidgetState ↔ Conversation**: One-to-One
- Widget state wraps a conversation
- State provides UI context (loading, error) for the conversation

**ChatMessage ↔ ChatMessage**: Sequential Order
- Messages are ordered by timestamp
- Each message is conceptually a response to previous context

---

## Storage Strategy

### Session-Based (React State - MVP)
- Conversation stored in React state (`useState`)
- Persists across page navigation (component mounted at Root level)
- Resets on page reload (acceptable for MVP)
- No localStorage persistence (future enhancement)

### Future Enhancement: localStorage (Not MVP)
```typescript
// Save conversation to localStorage after each message
localStorage.setItem('chatConversation', JSON.stringify(conversation));

// Load conversation on widget initialization
const savedConversation = localStorage.getItem('chatConversation');
if (savedConversation) {
  setConversation(JSON.parse(savedConversation));
}
```

### Future Enhancement: Backend Persistence (Not MVP)
- Requires user authentication/session ID
- POST to `/api/conversations` to save
- GET from `/api/conversations/{id}` to load
- Scope creep for MVP

---

## Type Definitions (TypeScript)

```typescript
// types.ts

export type MessageRole = 'user' | 'assistant';

export interface ChatMessage {
  id: string; // UUID or timestamp-based
  role: MessageRole;
  content: string; // 1-2000 chars for user, variable for assistant
  timestamp: string; // ISO 8601
  sources: string[]; // file paths, max 10
  error?: boolean; // optional, true if failed to send
}

export interface Conversation {
  id: string; // session ID
  messages: ChatMessage[];
  createdAt: string; // ISO 8601
  updatedAt: string; // ISO 8601
  isOpen: boolean;
}

export interface ChatWidgetState {
  conversation: Conversation;
  isLoading: boolean;
  error: string | null;
  inputValue: string; // max 2000 chars
  selectedText: string | null; // max 5000 chars
  selectedTextMode: boolean;
}

export interface ChatAPIRequest {
  question: string; // 1-2000 chars
}

export interface ChatSelectedAPIRequest extends ChatAPIRequest {
  selected_text: string; // 1-5000 chars
}

export interface ChatAPIResponse {
  answer: string;
  sources: string[]; // max 10
}

export interface APIError {
  type: 'network' | 'timeout' | 'server' | 'validation' | 'unknown';
  message: string; // user-friendly
  retry?: boolean; // true if user can retry
}
```

---

## Validation & Constraints

### Input Validation

| Field | Min | Max | Type | Required |
|-------|-----|-----|------|----------|
| question | 1 | 2000 | string | yes |
| selected_text | 1 | 5000 | string | yes (for /api/chat-selected) |
| answer | 1 | 10000 | string | yes (response) |
| sources | 0 | 10 | array | yes (response) |
| content (user) | 1 | 2000 | string | yes |
| content (assistant) | 1 | 10000 | string | yes |

### State Constraints

- `isLoading`: true → `error` must be null
- `selectedTextMode`: true → `selectedText` must be non-null
- `conversation.messages`: must be ordered by timestamp (ascending)
- `conversation.isOpen`: can be true/false independently of message count

---

**Status**: Phase 1 Design - Ready for Contracts & Quickstart

