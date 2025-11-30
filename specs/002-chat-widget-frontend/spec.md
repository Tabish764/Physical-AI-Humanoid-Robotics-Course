# Feature Specification: Interactive AI Chat Widget for Docusaurus Book

**Feature Branch**: `002-chat-widget-frontend`  
**Created**: 2025-11-29  
**Status**: Draft  
**Input**: User description: "Build a floating ChatGPT-style chat widget that appears on all pages of the Physical AI Docusaurus book. Users can click the chat icon to open a modal window, ask questions about the book content, and receive AI-generated answers with source citations from the FastAPI backend."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Ask Question About Book Content (Priority: P1)

A reader navigates to any page in the Physical AI textbook and encounters a floating chat icon in the bottom-right corner. The reader clicks the icon to open a chat modal, types a question like "What is ROS 2?", presses Enter, and receives an AI-generated answer with clickable links to source chapters.

**Why this priority**: This is the core value proposition of the feature—enabling readers to get instant answers about book content without leaving the page. Without this, the chat widget provides no value.

**Independent Test**: Can be fully tested by opening the book on any page, clicking the chat icon, typing a question, pressing send, and verifying that an answer with sources appears. Delivers immediate reader support.

**Acceptance Scenarios**:

1. **Given** the user is on any Docusaurus book page with the chat widget loaded, **When** the user clicks the floating chat icon, **Then** a chat modal opens smoothly with focus on the message input field.

2. **Given** the chat modal is open, **When** the user types a question and presses Enter, **Then** a loading spinner appears, the backend is called with the question, and the response displays with the answer and source links.

3. **Given** the backend returns an answer with sources, **When** the sources are displayed, **Then** they appear as clickable links that navigate to the correct book chapters when clicked.

4. **Given** the user is in an active chat conversation, **When** they send multiple messages, **Then** the conversation history shows all messages with proper timestamps and formatting (user messages right-aligned, AI messages left-aligned).

5. **Given** the backend takes longer than expected, **When** more than 5 seconds pass, **Then** a user-friendly error message appears with a retry button, not a loading spinner indefinitely.

---

### User Story 2 - Explain Selected Text from Book (Priority: P2)

A reader encounters a complex passage and selects a portion of text. A small "Ask AI" button appears near the selection. The reader clicks the button, and the chat modal opens with the selection pre-filled. The reader can add a question and submit it, receiving an explanation focused on the selected text.

**Why this priority**: This enhances the reading experience by allowing quick clarification of specific concepts without needing to rephrase or search. High value for learning, but secondary to basic chat functionality.

**Independent Test**: Can be fully tested by selecting text on a page, verifying the "Ask AI" button appears, clicking it, verifying the modal opens with pre-filled context, and confirming the explanation is returned.

**Acceptance Scenarios**:

1. **Given** the user selects text on a Docusaurus page, **When** the selection is made, **Then** a small "Ask AI" button appears near the selected text (top or bottom).

2. **Given** the user clicks the "Ask AI" button, **When** the click is registered, **Then** the chat modal opens with the selected text pre-filled in a context area and the input field ready for the user's question.

3. **Given** the user types a question and sends it, **When** the request is sent to `/api/chat-selected`, **Then** the response appears as an explanation focused on the selected text, with no external sources listed.

4. **Given** the chat modal is open with selected text context, **When** the user clears the input and types a new question, **Then** the selected text context remains available for follow-up questions.

---

### User Story 3 - Access Chat on Mobile (Priority: P1)

A user opens the Physical AI textbook on a mobile device (smartphone or tablet). The chat icon is visible and properly sized for touch (44px+ tap target). Clicking the icon opens a full-screen chat modal optimized for mobile. The user can type, send messages, and view responses without horizontal scrolling or layout issues.

**Why this priority**: Mobile users represent a significant portion of web traffic. The widget must be functional on all devices to provide value to the entire audience.

**Independent Test**: Can be fully tested by opening the book on a device < 768px width (or browser dev tools emulating mobile), clicking the chat icon, sending a message, and verifying the response is readable and the UI is responsive.

**Acceptance Scenarios**:

1. **Given** the user is on a mobile device (< 768px width), **When** the page loads, **Then** the chat icon is visible at 50px × 50px (or 45px × 45px) with appropriate spacing from edges.

2. **Given** the user clicks the chat icon on mobile, **When** the modal opens, **Then** it takes up the full screen or 90% of the viewport, with no horizontal overflow.

3. **Given** the user is typing a message on mobile, **When** the keyboard appears, **Then** the message input field remains visible and accessible (keyboard does not cover it).

4. **Given** the user sends a message on mobile, **When** the response appears, **Then** the text is readable (16px minimum font size), and the message list scrolls smoothly without jank.

---

### User Story 4 - Close and Reopen Chat (Priority: P1)

A user opens the chat widget, asks a question, receives a response, then closes the modal. The user navigates to another page in the book. When the user opens the chat widget again, the previous conversation history is still visible, allowing for continuity.

**Why this priority**: Persistence across navigation is essential for a seamless experience. Users expect chat history to be maintained during their session.

**Independent Test**: Can be fully tested by opening the chat, sending a message, closing it, navigating to a different page, reopening the chat, and verifying the history is still there.

**Acceptance Scenarios**:

1. **Given** the user sends a message and receives a response, **When** the user closes the modal (X button, Escape key, or clicking outside), **Then** the modal closes smoothly.

2. **Given** the user has closed the chat modal, **When** they navigate to a different page in the Docusaurus book, **Then** the chat component remains mounted in the background (no full reload/reset).

3. **Given** the user opens the chat widget again after navigation, **When** the modal opens, **Then** the previous conversation history is displayed (all messages and responses from this session).

4. **Given** the user has an active conversation, **When** they click the X button to close, **Then** the modal animates closed smoothly without jumping or flickering.

---

### Edge Cases

- **Empty or whitespace-only input**: If the user submits a message with only spaces or empty text, the system MUST reject it with a message like "Please enter a question" (not send to backend).

- **Extremely long input**: If the user pastes a very long text (> 2000 characters), the system MUST truncate or warn and prevent submission with a message like "Question is too long (max 2000 characters)".

- **Backend timeout**: If the backend does not respond within 30 seconds, the system MUST show "Request took too long. Please try again." with a retry button.

- **Network disconnected**: If the user loses internet connection during a request, the system MUST show "Check your internet connection" and allow retry.

- **Backend error (5xx)**: If the backend returns a 500-level error, the system MUST show "Failed to connect to AI. Please try again." without crashing.

- **Invalid response from backend**: If the backend returns a malformed response (missing `answer` or `sources` fields), the system MUST not crash; instead, log the error and show a generic error message.

- **Selected text too long**: If the user selects > 5000 characters and clicks "Ask AI", the system MUST truncate or warn: "Selected text is too long (max 5000 characters)".

- **Chat icon collision with page elements**: If a page has floating elements that overlap the chat icon position, the chat icon MUST remain on top (z-index: 9999) and clickable.

- **Rapid message submission**: If the user clicks send multiple times quickly, the system MUST debounce and only send one request, disabling the send button during the request.

- **No Qdrant results**: If the backend returns an answer but no sources, the system MUST display the answer with an empty sources list (not crash, not show error).

---

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: The system MUST display a floating chat icon button fixed at the bottom-right corner (20px from edges on desktop, responsive on mobile) on all Docusaurus pages.

- **FR-002**: The system MUST open a chat modal when the user clicks the chat icon, displaying the previous conversation history (if any) and an empty input field with focus.

- **FR-003**: The system MUST accept user text input (1-2000 characters) and send it to the backend via `POST /api/chat` with payload `{"question": "user text"}`.

- **FR-004**: The system MUST display the backend response (answer and sources) in the chat modal with the user message above and the AI response below in distinct visual containers.

- **FR-005**: The system MUST render sources as clickable links to book chapters, using file paths returned by the backend to navigate to the correct pages.

- **FR-006**: The system MUST display a loading spinner while waiting for a backend response and disable the send button during loading.

- **FR-007**: The system MUST display user-friendly error messages if the backend fails (network error, timeout, 5xx error, rate limit) and provide a retry button.

- **FR-008**: The system MUST close the chat modal when the user clicks the X button, presses Escape, or clicks outside the modal (if enabled).

- **FR-009**: The system MUST maintain conversation history across page navigation during a single user session (not require page reload to see previous messages).

- **FR-010**: The system MUST support mobile devices by rendering as a full-screen modal on screens < 768px wide and maintaining all functionality (typing, sending, scrolling).

- **FR-011**: The system MUST auto-scroll the message list to the newest message when a new message arrives or the user sends a message.

- **FR-012**: The system MUST detect user text selection on the page and display a small "Ask AI" button near the selection.

- **FR-013**: When the user clicks the "Ask AI" button, the system MUST open the chat modal with the selected text pre-filled and the input field ready for a question.

- **FR-014**: The system MUST send selected text requests to `POST /api/chat-selected` with payload `{"question": "user question", "selected_text": "selected content"}`.

- **FR-015**: The system MUST style the chat widget (icon, modal, messages, input) to match the Docusaurus theme and use CSS modules to avoid conflicts with site styles.

- **FR-016**: The system MUST support keyboard navigation (Tab through interactive elements, Enter to send, Escape to close) and announce new messages to screen readers.

- **FR-017**: The system MUST display message timestamps below each message in HH:MM format (e.g., "14:30").

- **FR-018**: The system MUST integrate with Docusaurus via `src/theme/Root.tsx` (theme swizzling) to render the chat widget globally on all pages.

- **FR-019**: The system MUST read the backend URL from environment variables or Docusaurus configuration (not hardcode it).

- **FR-020**: The system MUST trim user input (remove leading/trailing whitespace) before sending to the backend.

### Key Entities

- **Chat Message**: Represents a single message in the conversation
  - `role`: "user" | "assistant"
  - `content`: String (the message text)
  - `timestamp`: Date/time of the message
  - `sources`: Array of strings (source file paths, only for assistant messages)

- **Conversation State**: Represents the current chat session
  - `messages`: Array of Chat Messages
  - `isLoading`: Boolean (true while waiting for backend response)
  - `error`: String | null (error message if a request failed)
  - `isOpen`: Boolean (whether the modal is open)
  - `inputValue`: String (current text in the input field)

---

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Chat widget loads without errors on 100% of Docusaurus pages (no 404s, no crashed components).

- **SC-002**: Chat icon is visible and clickable on desktop (≥ 768px width) at 60px × 60px with smooth hover animation (1.05x scale).

- **SC-003**: Chat icon is visible and touch-friendly on mobile (< 768px width) at 50px × 50px with ≥ 44px tap target.

- **SC-004**: Chat modal opens smoothly (200-300ms animation) and closes smoothly (200-300ms animation) without flickering.

- **SC-005**: User can send a message and receive a response in under 5 seconds (normal conditions) with a loading indicator visible throughout.

- **SC-006**: Backend response is displayed correctly with the answer text and sources rendered as clickable links.

- **SC-007**: Source links navigate to the correct book chapter/page when clicked (verified by URL change or page scroll).

- **SC-008**: Conversation history persists across at least 5 page navigation events within the same session.

- **SC-009**: Chat widget is fully functional on mobile devices (< 768px width) with no horizontal overflow, unreadable text, or broken layout.

- **SC-010**: Error messages are user-friendly and actionable (e.g., "Check your internet connection" not "HTTP 503 Service Unavailable").

- **SC-011**: Chat widget does not interfere with existing Docusaurus functionality (navbar, sidebar, content navigation, search).

- **SC-012**: Chat widget bundle size is ≤ 50KB (minified + gzipped).

- **SC-013**: Initial load time of the chat widget is ≤ 100ms (time to interactive).

- **SC-014**: All interactive elements (button, input, close) are keyboard accessible and have visible focus indicators.

- **SC-015**: Chat widget works on Docusaurus 2.x and 3.x without modification or version-specific hacks.

- **SC-016**: Selected text feature is available when user selects > 1 character of text; "Ask AI" button appears within 100ms of selection.

- **SC-017**: No console errors or warnings appear when the chat widget is rendered and used normally.

- **SC-018**: 95% of API calls complete within 30 seconds; calls taking > 30 seconds are handled with a timeout error message.

---

## Assumptions

- **Backend is available**: The FastAPI backend is deployed and accessible at the configured URL during development and production.
- **Docusaurus configuration**: The Docusaurus site is using standard project structure with `src/` and `src/theme/` directories available for customization.
- **CORS is configured**: The backend has CORS headers set to allow requests from the Docusaurus frontend origin.
- **No authentication required**: For MVP, chat requests do not require user authentication; the backend accepts anonymous requests.
- **Session-based storage**: Conversation history is stored in the browser's React state during a session; no localStorage or backend persistence required for MVP.
- **Default backend URL**: If no backend URL is provided in configuration, the system defaults to `http://localhost:8000` (development environment).
- **User expects instant feedback**: Users expect the system to show a loading state immediately when they send a message and not keep them waiting without visual indication.

---

## Out of Scope (Non-Goals)

- User authentication or login system (handled separately if needed)
- Voice input or speech recognition
- File uploads or image sharing
- Real-time typing indicators ("User is typing...")
- Conversation history persistence across sessions (localStorage/backend)
- Dark mode toggle (uses existing Docusaurus theme)
- Message editing or deletion by user
- Emoji picker or rich text formatting
- Multi-language UI (English only for MVP)
- Sentiment analysis or feedback rating on responses
- Admin dashboard for monitoring chat usage
- Rate limiting UI (handled silently by backend with 503 response)

