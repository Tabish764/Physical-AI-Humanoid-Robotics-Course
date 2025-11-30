# API Contracts: Chat Widget Frontend

**Date**: 2025-11-29
**Feature**: Interactive AI Chat Widget for Docusaurus Book
**Base URL**: Configurable via environment variable (e.g., `http://localhost:8000` for dev)

## Overview

The Chat Widget communicates with the FastAPI backend via two REST endpoints:
1. **POST /api/chat** - Ask a question about book content (RAG search + LLM response)
2. **POST /api/chat-selected** - Explain selected text from the book

Both endpoints use JSON request/response format and follow standard REST conventions.

---

## Endpoint 1: POST /api/chat

### Description
Ask a question about the Physical AI textbook. The backend retrieves relevant content from Qdrant, sends it to Gemini LLM, and returns an answer with source citations.

### Request

**URL**: `POST {BASE_URL}/api/chat`

**Headers**:
```
Content-Type: application/json
```

**Body**:
```json
{
  "question": "What is ROS 2?"
}
```

**Validation**:
- `question`: string, required
  - Min length: 1 character
  - Max length: 2000 characters
  - Must not be empty or whitespace-only

**Example Valid Requests**:
```json
{
  "question": "What is ROS 2?"
}

{
  "question": "Explain the bipedal locomotion module in detail"
}

{
  "question": "How does URDF work with humanoids?"
}
```

**Example Invalid Requests**:
```json
{
  "question": ""
}

{
  "question": "   "
}

{
  "question": "a".repeat(2001)
}
```

### Response

**Success Response (200 OK)**:
```json
{
  "answer": "ROS 2 (Robot Operating System 2) is a middleware for robotics that provides tools and libraries for building robot applications. It uses a distributed architecture with nodes that communicate via topics, services, and actions...",
  "sources": [
    "module1-ros2/ros2-architecture.md",
    "module1-ros2/ros2-packages-python.md"
  ]
}
```

**Response Fields**:
- `answer`: string (required)
  - AI-generated answer to the question
  - Min length: 1 character
  - Max length: ~5000 characters (typical)
- `sources`: string[] (required)
  - Array of file paths from the book
  - Can be empty if no relevant sources found
  - Max 10 sources per response
  - Example: `["module1-ros2/ros2-architecture.md"]`

**Error Responses**:

### 400 Bad Request
```json
{
  "error": "Question is required and must be between 1 and 2000 characters"
}
```

### 503 Service Unavailable (Rate Limited)
```json
{
  "error": "Service is temporarily unavailable. Please try again later."
}
```
**Status Code**: 503 (too many requests, rate limit exceeded on Gemini)

### 500 Internal Server Error
```json
{
  "error": "Failed to process your question. Please try again."
}
```
**Status Code**: 500 (backend error, database error, LLM error)

### Network Timeout (30 seconds)
**Handling**: Client should set 30-second timeout on the request
```typescript
const controller = new AbortController();
const timeout = setTimeout(() => controller.abort(), 30000);
const response = await fetch(url, { signal: controller.signal });
```

---

## Endpoint 2: POST /api/chat-selected

### Description
Explain a specific piece of selected text from the book. The backend uses the provided text as context for the LLM response (no Qdrant search).

### Request

**URL**: `POST {BASE_URL}/api/chat-selected`

**Headers**:
```
Content-Type: application/json
```

**Body**:
```json
{
  "question": "Explain this in simple terms",
  "selected_text": "ROS 2 (Robot Operating System 2) is a middleware for robotics that provides tools and libraries for building robot applications."
}
```

**Validation**:
- `question`: string, required
  - Min length: 1 character
  - Max length: 2000 characters
  - Must not be empty or whitespace-only
- `selected_text`: string, required
  - Min length: 1 character
  - Max length: 5000 characters
  - The selected text from the book page

**Example Valid Requests**:
```json
{
  "question": "Explain this concept",
  "selected_text": "Bipedal locomotion is the ability to walk on two legs..."
}

{
  "question": "What does this mean?",
  "selected_text": "URDF (Unified Robot Description Format) is an XML format for specifying robot structure..."
}
```

**Example Invalid Requests**:
```json
{
  "question": "",
  "selected_text": "some text"
}

{
  "question": "Explain",
  "selected_text": ""
}

{
  "question": "Q",
  "selected_text": "a".repeat(5001)
}
```

### Response

**Success Response (200 OK)**:
```json
{
  "answer": "In simple terms, ROS 2 is software that helps robots work. It's like a translator between different robot parts, allowing them to talk to each other and coordinate their actions...",
  "sources": []
}
```

**Response Fields**:
- `answer`: string (required)
  - Explanation based on the provided selected text
  - Min length: 1 character
  - Max length: ~5000 characters
- `sources`: string[] (required, always empty for this endpoint)
  - Always an empty array (no external sources needed)

**Error Responses**:

### 400 Bad Request
```json
{
  "error": "Both 'question' and 'selected_text' are required and must be between 1 and 2000 characters (5000 for selected_text)"
}
```

### 503 Service Unavailable (Rate Limited)
```json
{
  "error": "Service is temporarily unavailable. Please try again later."
}
```

### 500 Internal Server Error
```json
{
  "error": "Failed to process your request. Please try again."
}
```

---

## Common Response Patterns

### Successful Chat Response
```
Status: 200 OK
Content-Type: application/json

{
  "answer": "...",
  "sources": [...]
}
```

### Rate Limit Exceeded
```
Status: 503 Service Unavailable
Content-Type: application/json

{
  "error": "Service is temporarily unavailable..."
}
```

### Validation Error
```
Status: 400 Bad Request
Content-Type: application/json

{
  "error": "Question is required and must be..."
}
```

### Server Error
```
Status: 500 Internal Server Error
Content-Type: application/json

{
  "error": "Failed to process..."
}
```

---

## Client Implementation Guidelines

### Success Path
```typescript
async function askQuestion(question: string) {
  try {
    const response = await fetch(`${BACKEND_URL}/api/chat`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ question })
    });

    if (!response.ok) {
      if (response.status === 503) {
        throw new Error('Service busy');
      } else if (response.status === 400) {
        const error = await response.json();
        throw new Error(error.error);
      } else {
        throw new Error('Backend error');
      }
    }

    const data = await response.json();
    return data; // { answer, sources }
  } catch (error) {
    // Handle error
  }
}
```

### Error Handling with Retry
```typescript
async function askQuestionWithRetry(
  question: string,
  maxRetries = 3
) {
  for (let attempt = 0; attempt < maxRetries; attempt++) {
    try {
      return await askQuestion(question);
    } catch (error) {
      if (attempt === maxRetries - 1) {
        throw error;
      }

      // Exponential backoff
      const delay = Math.pow(2, attempt) * 1000;
      await new Promise(r => setTimeout(r, delay));
    }
  }
}
```

### Timeout Handling
```typescript
async function askQuestionWithTimeout(question: string) {
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 30000);

  try {
    const response = await fetch(`${BACKEND_URL}/api/chat`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ question }),
      signal: controller.signal
    });
    clearTimeout(timeout);
    // ... handle response
  } catch (error) {
    if (error.name === 'AbortError') {
      throw new Error('Request timeout');
    }
    throw error;
  }
}
```

---

## CORS Requirements

The backend MUST set CORS headers to allow requests from the Docusaurus frontend:

```
Access-Control-Allow-Origin: http://localhost:3000 (dev) or https://example.com (prod)
Access-Control-Allow-Methods: POST, OPTIONS
Access-Control-Allow-Headers: Content-Type
```

The Docusaurus frontend will send `Origin` header, which the backend must explicitly allow.

---

## Rate Limiting (Backend Responsibility)

The backend enforces rate limits on the Gemini API (free tier: 10 requests per minute).

**Frontend Handling**:
1. Receive 503 response
2. Display user-friendly message: "Service busy. Please try again in a moment."
3. Disable send button for 5-10 seconds
4. Show retry button
5. Do not automatically retry (let user decide)

---

## Assumptions

1. **Backend is running** at the configured `BACKEND_URL`
2. **CORS is properly configured** on the backend
3. **API endpoints are available** at `/api/chat` and `/api/chat-selected` (exact paths)
4. **Response format is consistent** (always includes `answer` and `sources` fields)
5. **Timeout is acceptable at 30 seconds** (reasonable for Gemini API)
6. **No authentication required** for MVP (anonymous requests allowed)

---

## Future Enhancements

1. **WebSockets for streaming**: Real-time answer streaming as Gemini generates text
2. **Request pagination**: Handle very long conversations
3. **Message reactions**: Like/dislike feedback on responses
4. **Conversation sharing**: Share entire chat history
5. **Authentication**: User-specific chat history storage
6. **Analytics**: Track popular questions, user satisfaction

---

**Status**: Phase 1 Design - API Contracts Complete

