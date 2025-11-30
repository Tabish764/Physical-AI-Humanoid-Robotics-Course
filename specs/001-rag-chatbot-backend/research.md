# Research Findings

## Clarifications Addressed (Assumed Best Options):

### 1. Testing Framework/Strategy:

**Decision**: `pytest` will be extended for integration and contract testing. This will involve using `httpx` or similar libraries for making API requests within tests and employing mocking strategies where external dependencies (Qdrant, Neon, Gemini API) need to be isolated.

### 2. Performance Goals for Other Endpoints:

**Decisions**:
*   `/api/chat-selected`: Aim for a response time within **5 seconds**, similar to the `/api/chat` endpoint, as it also involves an LLM call.
*   `/api/health`: Target a response time of **<100ms**, given it's a simple status check of internal and external service connectivity.
*   `embed_book.py` script's processing time: The primary goal is correctness and successful completion. A reasonable target is to process a typical book (e.g., 50-100 markdown files) within **1 hour**.

### 3. Anticipated User Load/Data Volume for Scale/Scope:

**Decisions (MVP Scale)**:
*   **Concurrent Users**: Up to **10-20 concurrent users**.
*   **Requests per Second (RPS)**: Up to **1-2 RPS** for chat-related endpoints, keeping in mind the Gemini free tier rate limit of 10 RPM.
*   **Book Content Size**: The initial book content is assumed to be **50-100 markdown files**, with an average of 5-10 pages each.
*   **Chat History Growth**: Initially **modest**, with an expected growth to a **few thousand records** over time.