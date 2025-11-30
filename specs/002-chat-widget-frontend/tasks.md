# Implementation Tasks: Chat Widget Frontend

**Feature**: Interactive AI Chat Widget for Docusaurus Book  
**Branch**: `002-chat-widget-frontend`  
**Date**: 2025-11-29  
**Status**: Phase 2 (Task Breakdown)

---

## Overview

This document breaks down the Chat Widget Frontend specification into executable implementation tasks organized by user story. Each task is independently testable and includes specific file paths for completion.

**User Stories**:
1. **US1** (P1): Ask Question About Book Content
2. **US2** (P2): Explain Selected Text from Book
3. **US3** (P1): Access Chat on Mobile
4. **US4** (P1): Close and Reopen Chat

**Phase Structure**:
- **Phase 1**: Setup (project initialization)
- **Phase 2**: Foundational (blocking prerequisites)
- **Phase 3**: User Story 1 - Core Chat (P1)
- **Phase 4**: User Story 2 - Selected Text (P2)
- **Phase 5**: User Story 3 - Mobile (P1)
- **Phase 6**: User Story 4 - Persistence (P1)
- **Phase 7**: Polish & Cross-Cutting Concerns

---

## Dependency Graph & Execution Order

```
Phase 1: Setup
    ↓
Phase 2: Foundational (blocking all user stories)
    ├─→ Phase 3: US1 (Core Chat) [can parallel with others after Phase 2]
    ├─→ Phase 4: US2 (Selected Text) [depends on Phase 3]
    ├─→ Phase 5: US3 (Mobile) [depends on Phase 3]
    └─→ Phase 6: US4 (Persistence) [depends on Phase 3]
           ↓
Phase 7: Polish & Cross-Cutting Concerns
```

**Execution Recommendation**:
- **MVP Scope** (minimum viable product): Complete Phase 1 + 2 + 3 (~30 tasks)
- **Recommended First Release**: Phase 1 + 2 + 3 + 5 (~40 tasks) - adds mobile support
- **Full Feature**: All phases (~70 tasks) - all user stories complete

---

## Phase 1: Setup & Project Initialization

### Goal
Initialize the React component project structure, dependencies, and Docusaurus integration points.

### Independent Test Criteria
- Project structure created as specified
- All dependencies installed without errors
- TypeScript compilation succeeds
- Docusaurus build succeeds without breaking existing site

### Setup Tasks

- [x] T001 Create project directory structure at `src/components/ChatWidget/` with subdirectories for hooks, services, types, and styles
- [x] T002 Create `src/components/ChatWidget/index.ts` to export all public components
- [x] T003 Create `src/hooks/useChat.ts` stub for chat API logic hook (empty implementation)
- [x] T004 Create `src/hooks/useSelectedText.ts` stub for selected text detection hook (empty implementation)
- [x] T005 Create `src/services/api.ts` stub for REST API client functions (empty implementation)
- [x] T006 Create `src/types/chat.ts` with all TypeScript interfaces (ChatMessage, Conversation, ChatWidgetState, API types)
- [x] T007 Create `.env.local` template with `REACT_APP_BACKEND_URL=http://localhost:8000` example
- [ ] T008 Update `src/theme/Root.tsx` to import and render ChatWidget globally (add mount point)
- [x] T009 [P] Create CSS module files: `src/components/ChatWidget/ChatWidget.module.css`, `src/components/ChatWidget/ChatIcon.module.css`, `src/components/ChatWidget/ChatModal.module.css`, `src/components/ChatWidget/MessageList.module.css`, `src/components/ChatWidget/Message.module.css`, `src/components/ChatWidget/MessageInput.module.css`
- [x] T010 [P] Create component stub files with React.FC signature and empty JSX: ChatWidget.tsx, ChatIcon.tsx, ChatModal.tsx, MessageList.tsx, Message.tsx, MessageInput.tsx in `src/components/ChatWidget/`
- [ ] T011 Verify TypeScript strict mode compiles all files without errors in `src/components/ChatWidget/`
- [ ] T012 Verify Docusaurus build completes successfully with chat widget integrated
- [x] T013 Create `src/components/ChatWidget/__tests__/` directory for unit tests
- [x] T014 Create test setup file `src/components/ChatWidget/__tests__/setup.ts` with jest configuration helpers

---

## Phase 2: Foundational Infrastructure (Blocking for All User Stories)

### Goal
Implement shared utilities, API client, TypeScript types, and base styling that all user stories depend on.

### Independent Test Criteria
- API client functions work (can call both endpoints)
- TypeScript compilation in strict mode succeeds
- All types are properly exported and importable
- Base CSS variables and reset styles load without conflicts

### Foundational Tasks

- [ ] T015 [P] Implement all TypeScript types in `src/types/chat.ts`: ChatMessage, Conversation, ChatWidgetState, APIRequest, APIResponse (with complete JSDoc comments)
- [ ] T016 [P] Create `src/types/docusaurus.ts` for Docusaurus-specific types (useDocusaurusContext, useBaseUrl, etc.)
- [ ] T017 [P] Implement REST API client in `src/services/api.ts`:
  - `async postChat(question: string): Promise<APIResponse>` 
  - `async postChatSelected(question: string, selectedText: string): Promise<APIResponse>`
  - Error handling (timeout at 30s, user-friendly error messages)
  - Retry logic with exponential backoff (3 retries max)
  - CORS error handling
  - Request/response logging
- [ ] T018 [P] Create environment configuration module `src/config/api.ts`:
  - `getBackendUrl(): string` function
  - Read from REACT_APP_BACKEND_URL or process.env.REACT_APP_BACKEND_URL
  - Validate URL format
  - Throw error if backend URL not configured
- [ ] T019 Implement `src/hooks/useChat.ts` hook with state management:
  - `const { messages, isLoading, error, sendMessage, clearConversation } = useChat()`
  - State: conversation messages, loading flag, error message
  - Methods: sendMessage(question), clearConversation()
  - Calls api.postChat() and handles response/error
  - Use localStorage for session ID (optional, not required for MVP)
- [ ] T020 Implement `src/hooks/useSelectedText.ts` hook for text selection detection:
  - `const { selectedText, isSelectionActive, clearSelection } = useSelectedText()`
  - Detect document.onmouseup / document.ontouchend events
  - Extract window.getSelection().toString()
  - Handle edge case: selection across multiple elements
  - Cleanup on unmount
- [ ] T021 Create global base styles in `src/components/ChatWidget/base.module.css`:
  - CSS reset for chat components
  - Docusaurus theme variable mappings (primary color, secondary color, background)
  - Accessibility: focus-visible styles, high contrast support
  - Mobile breakpoint: 768px
  - Z-index management (widget: 9999)
- [ ] T022 Create utility functions in `src/utils/formatting.ts`:
  - `formatTimestamp(date: Date): string` → HH:MM format
  - `truncateText(text: string, maxLength: number): string` → "..." suffix if truncated
  - `sanitizeText(text: string): string` → trim, remove extra whitespace
  - `validateQuestion(text: string): { valid: boolean; error?: string }`
- [ ] T023 Create utility functions in `src/utils/accessibility.ts`:
  - `generateId(prefix: string): string` → unique IDs for aria-labelledby, aria-describedby
  - `announceToScreenReader(message: string): void` → aria-live region updates
  - `trapFocus(element: HTMLElement): void` → focus trap in modal
- [ ] T024 Create test utilities in `src/__tests__/mocks.ts`:
  - Mock api.postChat(), api.postChatSelected()
  - Mock getBackendUrl()
  - Mock window.getSelection()
  - MSW setup for API mocking (optional)
- [ ] T025 Create test file `src/services/__tests__/api.test.ts`:
  - Test postChat() success path
  - Test postChatSelected() success path
  - Test timeout handling (30s)
  - Test network error handling
  - Test retry logic
  - Test CORS error handling
  - Test error message formatting
- [ ] T026 Create test file `src/hooks/__tests__/useChat.test.ts`:
  - Test initial state
  - Test sendMessage() updates state
  - Test error state on API failure
  - Test loading state during request
  - Test clearConversation() resets state
- [ ] T027 Create test file `src/hooks/__tests__/useSelectedText.test.ts`:
  - Test selection detection on mouseup
  - Test selection detection on touchend
  - Test empty selection handling
  - Test cross-element selection
  - Test cleanup on unmount

---

## Phase 3: User Story 1 - Ask Question About Book Content (P1)

### Goal
Implement core chat functionality: floating icon, modal, message display, and API integration for asking questions.

### Independent Test Criteria
- User can click floating icon and open modal
- User can type question and send it
- Backend response displays with answer and sources
- Sources render as clickable links
- Loading state visible during request
- Error messages display on failure
- Modal closes smoothly

### Story 1 Tasks: UI Components

- [ ] T028 [P] [US1] Implement `src/components/ChatWidget/ChatIcon.tsx`:
  - Floating button at bottom-right (60px × 60px desktop, 50px × 50px mobile)
  - SVG icon or emoji (💬)
  - Hover effect: 1.05x scale, opacity transition
  - Accessibility: aria-label="Open chat", role="button", tabindex="0", keyboard support (Enter, Space)
  - Props: onClick, isOpen (show different icon if modal is open)
  - Styles imported from ChatIcon.module.css
  - Z-index: 9999
  - Fixed positioning, 20px from bottom and right (responsive)
- [ ] T029 [US1] Implement `src/components/ChatWidget/ChatModal.tsx`:
  - Modal wrapper with backdrop (semi-transparent black)
  - Modal dimensions: 400px × 600px on desktop, full-screen on mobile (< 768px)
  - Close button (X icon, top-right)
  - Title: "Ask about the book"
  - Header, content area, footer (input area)
  - Accessibility: role="dialog", aria-modal="true", aria-labelledby="modal-title"
  - Keyboard: Escape to close
  - Animation: slide-up 200-300ms
  - Props: isOpen, onClose, children
  - Styles imported from ChatModal.module.css
  - Focus trap on open (useEffect cleanup)
- [ ] T030 [US1] Implement `src/components/ChatWidget/MessageList.tsx`:
  - Display array of ChatMessage objects
  - Auto-scroll to bottom on new message (useEffect with scrollRef)
  - Message containers: user messages right-aligned, assistant messages left-aligned
  - Placeholder text when empty: "Start a conversation by typing a question..."
  - Props: messages: ChatMessage[], isLoading: boolean
  - Styles imported from MessageList.module.css
  - Smooth scrolling, no jank on mobile
  - Loading indicator at bottom (spinner) when isLoading=true
- [ ] T031 [US1] Implement `src/components/ChatWidget/Message.tsx`:
  - Render single ChatMessage
  - Display: role badge (User/AI), content text, timestamp
  - Timestamp format: HH:MM (use formatTimestamp utility)
  - Assistant messages: sources section below (clickable links)
  - Error state: red background, error icon if message.error=true
  - Props: message: ChatMessage, onSourceClick: (path: string) => void
  - Styles imported from Message.module.css
  - Accessibility: role="article", aria-label for screen readers
- [ ] T032 [US1] Implement `src/components/ChatWidget/MessageInput.tsx`:
  - Text input field (multiline, max 2000 chars)
  - Character counter: "XXX / 2000" below input
  - Send button (paper plane icon or "Send" text)
  - Disabled while isLoading=true
  - Validation: disable send if empty or whitespace-only
  - Props: value: string, onChange, onSubmit, isLoading: boolean, error?: string
  - Error message display below input (red text)
  - Placeholder: "Ask a question..."
  - Keyboard: Enter to send, Shift+Enter for newline
  - Styles imported from MessageInput.module.css
  - Accessibility: aria-label="Message input", aria-describedby for error messages
- [ ] T033 [P] [US1] Implement styles in `src/components/ChatWidget/ChatIcon.module.css`:
  - Floating button styling (border-radius: 50%, fixed position)
  - Hover and focus states
  - SVG/emoji sizing and centering
  - Mobile responsive: size change at 768px breakpoint
  - Dark mode support (use CSS variables from Docusaurus theme)
  - Shadow for depth
- [ ] T034 [P] [US1] Implement styles in `src/components/ChatWidget/ChatModal.module.css`:
  - Modal backdrop (position: fixed, 0 0 100% 100%, z-index: 9998)
  - Modal container (background, border-radius, box-shadow)
  - Responsive dimensions (400×600 desktop, 100% mobile)
  - Header (title, close button positioning)
  - Content scrollable area
  - Footer (input area)
  - Animation classes: slideUp, fadeIn
  - Mobile: full screen or 90% viewport
- [ ] T035 [P] [US1] Implement styles in `src/components/ChatWidget/MessageList.module.css`:
  - Container scrolling (max-height, overflow-y auto)
  - Message bubbles (user right, assistant left)
  - User bubble: primary color background, white text
  - Assistant bubble: light gray background, dark text
  - Spacing and alignment (padding, margin)
  - Loading spinner styling
  - Smooth scroll behavior
- [ ] T036 [P] [US1] Implement styles in `src/components/ChatWidget/Message.module.css`:
  - Message bubble layout (flexbox, direction based on role)
  - Timestamp: small, gray, below message
  - Sources section: list of links
  - Link styling: underline, hover color change
  - Error state: red border, red text
  - Avatar or role indicator (optional)
- [ ] T037 [P] [US1] Implement styles in `src/components/ChatWidget/MessageInput.module.css`:
  - Input field styling (border, focus state, padding)
  - Textarea grow with content (or fixed height with scroll)
  - Character counter (right-aligned, small font)
  - Send button styling (primary color, hover, disabled state)
  - Error message styling (red text)
  - Keyboard indicator: "Shift+Enter for newline" hint
  - Mobile: full-width, larger touch targets

### Story 1 Tasks: Integration

- [ ] T038 [US1] Implement `src/components/ChatWidget/ChatWidget.tsx` (main container):
  - Use useChat() and useSelectedText() hooks
  - State: isOpen (modal open/close), selectedText
  - Render ChatIcon + ChatModal
  - Pass messages, isLoading, error to child components
  - Handle sendMessage from MessageInput
  - Handle source clicks (navigate to chapter)
  - Manage modal open/close
  - Props: none (self-contained)
  - Styles imported from ChatWidget.module.css
  - Mount at root level in src/theme/Root.tsx
- [ ] T039 [P] [US1] Implement styles in `src/components/ChatWidget/ChatWidget.module.css`:
  - Container styling (position, display)
  - Z-index management for stacked components
  - Global overrides or resets (if needed)
  - Animation timing (200-300ms)
- [ ] T040 [US1] Update `src/theme/Root.tsx` to render ChatWidget:
  - Import ChatWidget from src/components/ChatWidget
  - Add mount point at root level (after main content, before closing root element)
  - Wrap with error boundary (optional but recommended)
  - Pass any necessary props (backend URL, theme context)
- [ ] T041 [US1] Implement source link navigation in Message component:
  - onClick handler: extract file path from source
  - Use useDocusaurusContext().siteConfig.baseUrl to construct full path
  - Navigate using window.location.href or router (Docusaurus router if available)
  - Handle malformed paths gracefully (show error message)
- [ ] T042 [US1] Create unit tests for ChatIcon component `src/components/ChatWidget/__tests__/ChatIcon.test.tsx`:
  - Test render and visibility
  - Test click handler
  - Test hover effect
  - Test accessibility (aria-label, keyboard support)
  - Test isOpen prop changes icon style
- [ ] T043 [US1] Create unit tests for ChatModal component `src/components/ChatWidget/__tests__/ChatModal.test.tsx`:
  - Test render when isOpen=true/false
  - Test close button click
  - Test Escape key closes
  - Test backdrop click (if enabled)
  - Test focus trap
  - Test accessibility (role="dialog", aria-modal)
- [ ] T044 [US1] Create unit tests for MessageList component `src/components/ChatWidget/__tests__/MessageList.test.tsx`:
  - Test render empty state
  - Test render multiple messages
  - Test user vs assistant message styling
  - Test auto-scroll on new message
  - Test loading spinner visibility
- [ ] T045 [US1] Create unit tests for Message component `src/components/ChatWidget/__tests__/Message.test.tsx`:
  - Test render user message
  - Test render assistant message with sources
  - Test timestamp formatting
  - Test source links render and are clickable
  - Test error state styling
  - Test accessibility
- [ ] T046 [US1] Create unit tests for MessageInput component `src/components/ChatWidget/__tests__/MessageInput.test.tsx`:
  - Test input value changes
  - Test character counter updates
  - Test send button disabled when empty
  - Test Enter key submits
  - Test Shift+Enter adds newline
  - Test error message display
  - Test max length validation
- [ ] T047 [US1] Create integration test for ChatWidget `src/components/ChatWidget/__tests__/ChatWidget.integration.test.tsx`:
  - Test full flow: click icon → modal opens → type question → send → response displays
  - Test message history accumulates
  - Test error handling: failed request shows error message with retry button
  - Test close modal and reopen preserves history (for this phase, just within component)
- [ ] T048 [US1] Create E2E test with Playwright `e2e/chat-widget.spec.ts` (or Cypress):
  - Test user can click chat icon and modal opens
  - Test user can type and send question
  - Test response displays with answer and sources
  - Test click source link navigates to chapter
  - Test close modal with X button
  - Test close modal with Escape key
  - Test loading spinner shows during request
  - Test error message shows on network error
  - Run on desktop viewport (1280×1024)

---

## Phase 4: User Story 2 - Explain Selected Text from Book (P2)

### Goal
Implement selected text detection, "Ask AI" button overlay, and integration with chat for explaining selections.

### Independent Test Criteria
- User can select text and see "Ask AI" button appear
- Clicking "Ask AI" opens modal with selected text pre-filled
- Sending request includes both question and selected_text
- Backend response displays as explanation
- Selected text context persists for follow-up questions

### Story 2 Tasks: UI Components

- [x] T049 [P] [US2] Implement `src/components/ChatWidget/SelectionButton.tsx`:
  - Small "Ask AI" button that appears near selected text
  - Positioned near selection using getBoundingClientRect()
  - Styles: compact button, icon + "Ask AI" text or just icon
  - Props: position: { x: number; y: number }, onClick
  - Accessibility: aria-label="Ask AI about selected text"
  - Animation: fade in 100ms
  - Z-index: 9998 (below modal, above content)
- [x] T050 [P] [US2] Implement styles in `src/components/ChatWidget/SelectionButton.module.css`:
  - Absolute positioning based on props.position
  - Button styling (small, compact)
  - Hover effect
  - Fade-in animation
  - Shadow for visibility over content
  - Mobile: button larger (48px minimum)

### Story 2 Tasks: Integration

- [x] T051 [US2] Enhance `useSelectedText()` hook to return position:
  - `const { selectedText, isSelectionActive, position, clearSelection } = useSelectedText()`
  - Calculate position: (x, y) from selection.getBoundingClientRect()
  - Position button slightly above/below selection
  - Handle edge case: selection near viewport edges (reposition)
  - Return null if selection near modal (to avoid overlap)
- [x] T052 [US2] Update ChatWidget to render SelectionButton:
  - Conditionally render SelectionButton when selectedText is active
  - Pass position from useSelectedText()
  - onClick: set selectedTextMode=true, open modal
- [x] T053 [US2] Enhance ChatModal to show selected text context:
  - New prop: selectedText?: string
  - Display selected text in gray box with icon (optional)
  - Show below modal title
  - Include in submission to backend via postChatSelected()
- [x] T054 [US2] Enhance MessageInput for selected text mode:
  - Conditional rendering: if selectedTextMode=true, show context box
  - Placeholder changes to: "Ask a follow-up question..."
  - Buttons: "Ask" or "Clear selection"
  - Clear selection handler: reset selectedText, revert to normal mode
- [x] T055 [US2] Implement selected text submission in useChat():
  - Enhance sendMessage() to check if selectedTextMode=true
  - If true, call api.postChatSelected(question, selectedText)
  - If false, call api.postChat(question)
  - Response handling same for both
- [ ] T056 [US2] Create unit tests for SelectionButton `src/components/ChatWidget/__tests__/SelectionButton.test.tsx`:
  - Test render at correct position
  - Test click handler
  - Test fade-in animation
  - Test accessibility
- [ ] T057 [US2] Create unit tests for selected text flow `src/components/ChatWidget/__tests__/ChatWidget.selectedText.test.tsx`:
  - Test selection detection
  - Test button appears at correct position
  - Test click button opens modal with context
  - Test selected text passes to API request
  - Test response displays
  - Test clear selection resets mode
- [ ] T058 [US2] Create E2E test for selected text `e2e/chat-widget-selected-text.spec.ts`:
  - Test select text on page
  - Test "Ask AI" button appears
  - Test click button opens modal
  - Test modal shows selected text context
  - Test send question with selected text
  - Test response displays
  - Test clear selection button resets
  - Run on desktop viewport

---

## Phase 5: User Story 3 - Access Chat on Mobile (P1)

### Goal
Ensure chat widget is fully functional and responsive on mobile devices (< 768px width).

### Independent Test Criteria
- Chat icon visible and touch-friendly on mobile (50px with 44px+ tap target)
- Modal renders full-screen or 90% viewport on mobile
- No horizontal scrolling or layout issues
- Input field remains visible when keyboard appears
- Text is readable (≥16px)
- All interactions work on touch devices

### Story 3 Tasks: Responsive Design

- [ ] T059 [P] [US3] Enhance ChatIcon styling for mobile in `ChatIcon.module.css`:
  - Add media query for < 768px
  - Size: 50px × 50px (from 60px × 60px)
  - Spacing: 15px from edges (from 20px)
  - Bottom: calc(env(safe-area-inset-bottom) + 15px) for notched phones
  - Ensure tap target ≥ 44px (add larger clickable area with ::before)
  - Test: 50px button + 8px padding around = 66px × 66px clickable area
- [ ] T060 [P] [US3] Enhance ChatModal styling for mobile in `ChatModal.module.css`:
  - Add media query for < 768px
  - Width: 100% or calc(100% - 16px) with 8px margin (no horizontal overflow)
  - Height: 90vh (90% viewport height, leaves room for keyboard)
  - Bottom positioning: 0 (full-screen from bottom)
  - Use max-height instead of fixed height for iOS keyboard behavior
  - Ensure header and input area remain visible
  - Use CSS env() variables for safe areas (notched devices)
- [ ] T061 [P] [US3] Enhance MessageList styling for mobile in `MessageList.module.css`:
  - Add media query for < 768px
  - Adjust padding and margins for mobile spacing
  - Ensure smooth scrolling on iOS (-webkit-overflow-scrolling: touch)
  - Test: no jank when scrolling with messages
- [ ] T062 [P] [US3] Enhance Message styling for mobile in `Message.module.css`:
  - Add media query for < 768px
  - Increase padding slightly for better touch targets
  - Ensure text is ≥16px font size
  - Adjust margins between messages
- [ ] T063 [P] [US3] Enhance MessageInput styling for mobile in `MessageInput.module.css`:
  - Add media query for < 768px
  - Input height: ≥44px (touch target)
  - Send button: ≥44px × 44px (touch target)
  - Remove fixed height if used; allow textarea to grow with content
  - Test: input field remains above keyboard on iOS/Android
  - Use position: fixed or absolute for footer to stay above keyboard
- [ ] T064 [P] [US3] Enhance SelectionButton styling for mobile in `SelectionButton.module.css`:
  - Add media query for < 768px
  - Button: ≥48px × 48px
  - Adjust positioning to avoid overlapping modal or keyboard
  - Test on real mobile devices

### Story 3 Tasks: Mobile-Specific Behavior

- [ ] T065 [US3] Add mobile viewport meta tags to Docusaurus config (if not present):
  - Ensure `<meta name="viewport" content="width=device-width, initial-scale=1.0, viewport-fit=cover">` is set
  - Enable safe-area-inset for notched devices
- [ ] T066 [US3] Enhance MessageInput component for mobile keyboard behavior:
  - On mobile, when input focused: scroll to input in view
  - Handle iOS keyboard auto-zoom (font-size ≥16px on input)
  - Test: keyboard does not cover input field
- [ ] T067 [US3] Update ChatModal to handle soft keyboard on mobile:
  - On mobile, detect keyboard open (window.innerHeight change)
  - Adjust modal max-height when keyboard visible
  - Use window.visualViewport API (or resize observer)
  - Test: modal scrolls if needed, input stays visible
- [ ] T068 [US3] Add touch-specific event handling:
  - Use onTouchStart / onTouchEnd in addition to onClick
  - Enhance useSelectedText() to detect selection on mobile (works differently than desktop)
  - Test on iOS: selection behavior may differ (context menu vs actual selection)
  - For iOS, may need text interaction API or workaround
- [ ] T069 [US3] Create mobile-specific E2E tests `e2e/chat-widget.mobile.spec.ts`:
  - Test on iPhone 12 (390×844) emulation
  - Test on iPad (768×1024) emulation
  - Test on Android (360×800) emulation
  - Verify icon visibility and clickability
  - Verify modal opens full-screen
  - Verify no horizontal overflow
  - Verify input field visible above keyboard
  - Verify text is readable
  - Verify touch interactions work
  - Verify scroll smooth on message list
- [ ] T070 [US3] Create accessibility test for mobile `e2e/chat-widget.a11y.mobile.spec.ts`:
  - Test VoiceOver on iOS (if available)
  - Test TalkBack on Android (if available)
  - Verify all buttons and inputs are announced
  - Verify focus is manageable via keyboard
  - Verify color contrast on mobile (≥4.5:1)
- [ ] T071 [US3] Create unit tests for mobile responsiveness `src/components/ChatWidget/__tests__/ChatWidget.mobile.test.tsx`:
  - Mock window.matchMedia for < 768px
  - Test ChatIcon size and spacing on mobile
  - Test ChatModal dimensions on mobile
  - Test MessageInput height adjustment
  - Test touch event handling

---

## Phase 6: User Story 4 - Close and Reopen Chat with Persistence (P1)

### Goal
Implement conversation history persistence across page navigation and smooth modal open/close animations.

### Independent Test Criteria
- Modal opens and closes smoothly (200-300ms animation)
- Conversation history persists when modal closed
- Navigation to different page preserves conversation
- Reopening modal shows full history
- Component remains mounted during navigation
- No memory leaks from event listeners

### Story 4 Tasks: State Persistence

- [ ] T072 [US4] Enhance `useChat()` hook with session persistence:
  - Store conversation in localStorage under key: `chat_widget_session_${sessionId}`
  - On first render, check localStorage and restore conversation if exists
  - On every message added, save to localStorage
  - Session ID: generated on first mount (UUID or timestamp-based)
  - Clear on browser close (optional) or persist for 24 hours (configurable)
  - Validate persisted data format before using
- [ ] T073 [US4] Implement conversation clearing:
  - Add `clearConversation()` method to useChat() hook
  - Clears localStorage for current session
  - Resets messages array to empty
  - Called when user clicks "New Conversation" or similar
  - Confirm dialog before clearing (optional)
- [ ] T074 [US4] Update ChatWidget to manage mount lifecycle:
  - Ensure ChatWidget mounted at root level (src/theme/Root.tsx) to persist across navigation
  - Component should NOT re-mount on page navigation
  - Test: verify component instance persists (e.g., console.log on mount)
  - If page routing causes re-mount, use useRef or context to maintain state across mounts
- [ ] T075 [US4] Create custom hook `useConversationPersistence()`:
  - Wrapper around useChat() that handles localStorage sync
  - Auto-saves on every message change (debounced, 500ms)
  - Auto-restores on mount
  - Provides clearConversation() method
- [ ] T076 [US4] Enhance api.ts to track request IDs for idempotency:
  - Generate unique requestId for each API call
  - Send in header: X-Request-ID
  - Use to deduplicate retries (backend can use this)
  - Store requestId with message for audit trail

### Story 4 Tasks: UI/UX Polish

- [ ] T077 [P] [US4] Enhance modal animations in ChatModal.module.css:
  - Add keyframe animations: @keyframes slideUp, @keyframes fadeIn
  - slideUp: 0% transform: translateY(100%), 100% transform: translateY(0%)
  - Duration: 300ms ease-out
  - Also animate on close: slideDown / fadeOut
  - Test: smooth 60fps animation, no jank
- [ ] T078 [P] [US4] Add loading state animations in MessageList.module.css:
  - Spinner animation: rotating loader or pulsing dots
  - Duration: 1.5s infinite
  - Smooth rotation or pulsing
  - Test: visible and smooth on both desktop and mobile
- [ ] T079 [US4] Implement bounce animation for new messages in Message.module.css:
  - New messages (assistant responses) animate in: @keyframes slideInLeft
  - Duration: 200ms ease-out
  - Offset: 20px from left
  - Only for new messages, not on page load
  - Test: smooth entry, no jank
- [ ] T080 [US4] Add visual feedback for interactions:
  - Send button: active state (button press animation)
  - Message hover: subtle background color change
  - Sources hover: underline and color change
  - Test: feedback visible and responsive
- [ ] T081 [P] [US4] Enhance MessageInput styling for feedback:
  - Add active/focus state with border color change
  - Character counter color changes near limit (orange at 1800, red at 1900)
  - Visual feedback when text pasted
  - Test: all states visible and clear

### Story 4 Tasks: Testing

- [ ] T082 [US4] Create integration test for persistence `src/components/ChatWidget/__tests__/ChatWidget.persistence.test.tsx`:
  - Test localStorage save on message add
  - Test localStorage restore on mount
  - Test persistence across component unmount/remount
  - Test clearing conversation clears localStorage
  - Mock localStorage API
- [ ] T083 [US4] Create E2E test for full user flow `e2e/chat-widget.full-flow.spec.ts`:
  - Test: click icon → open modal
  - Test: type question → send → response displays
  - Test: message appears in history
  - Test: close modal (X button)
  - Test: navigate to different page
  - Test: reopen modal → history still there
  - Test: close modal (Escape key)
  - Test: navigate → reopen → history persists
  - Test: multiple messages → all in history
  - Test: clear conversation → history gone
  - Run on desktop viewport
- [ ] T084 [US4] Create E2E test for animations `e2e/chat-widget.animations.spec.ts`:
  - Test modal open animation: 200-300ms
  - Test modal close animation: 200-300ms
  - Test new message slide-in animation
  - Test loading spinner animation
  - Verify no jank (60fps)
  - Use Playwright performance API if available

### Story 4 Tasks: Error Handling & Edge Cases

- [ ] T085 [US4] Enhance error handling for persistence:
  - Handle localStorage quota exceeded (QuotaExceededError)
  - Fall back to session state if localStorage fails
  - Show warning message: "Chat history not saved locally"
  - Test: on browsers with disabled localStorage
- [ ] T086 [US4] Add error recovery mechanism:
  - If localStorage corrupted, clear it and start fresh
  - Validate persisted data format before using
  - Log errors for debugging
  - Test: manually corrupt localStorage entry
- [ ] T087 [US4] Implement graceful degradation:
  - If component fails to initialize, show fallback UI
  - Error boundary catches crashes
  - User can dismiss error and retry
  - Test: throw error from useChat(), verify error boundary catches it

---

## Phase 7: Polish & Cross-Cutting Concerns

### Goal
Add testing, documentation, performance optimization, and final quality assurance.

### Polish Tasks: Documentation

- [ ] T088 Create component documentation in `src/components/ChatWidget/README.md`:
  - Overview and purpose
  - Component structure diagram
  - Props for each component
  - Usage examples
  - Customization guide (colors, sizes, text)
  - Troubleshooting section
- [ ] T089 Create hooks documentation in `src/hooks/README.md`:
  - `useChat()` hook: parameters, return values, examples
  - `useSelectedText()` hook: parameters, return values, examples
  - `useConversationPersistence()` hook: parameters, return values, examples
  - Error handling
  - Best practices
- [ ] T090 Create API documentation in `src/services/README.md`:
  - `api.postChat()` function: parameters, return values, errors
  - `api.postChatSelected()` function: parameters, return values, errors
  - Retry logic behavior
  - Timeout handling
  - Environment configuration
- [ ] T091 Create developer guide in `specs/002-chat-widget-frontend/DEVELOPER.md`:
  - Development setup (Node.js, npm install)
  - Running dev server (npm start)
  - Running tests (npm test)
  - Running E2E tests (npm run test:e2e)
  - Building for production (npm run build)
  - Debugging tips (React DevTools, Network tab)
  - Common issues and solutions

### Polish Tasks: Performance & Optimization

- [ ] T092 [P] Implement code splitting for ChatWidget:
  - Use React.lazy() and Suspense to lazy-load ChatWidget (optional)
  - Bundle size target: ≤ 50KB (minified + gzipped)
  - Measure bundle: npm run build, check dist size
  - If > 50KB, identify heavy dependencies (lodash, moment, etc.)
  - Minify and optimize as needed
- [ ] T093 [P] Implement performance monitoring:
  - Add Web Vitals tracking (LCP, FID, CLS)
  - Monitor API response times
  - Track user interactions (time to open modal, time to send)
  - Log metrics to console or analytics service
  - Create performance budget: initial load < 100ms, modal open < 300ms
- [ ] T094 [P] Optimize images and assets:
  - Ensure SVG icons are optimized (minimal paths)
  - Use CSS instead of images where possible
  - Test loading on slow 3G network (Chrome DevTools)
- [ ] T095 Implement memoization for performance:
  - Use React.memo() for components that don't need re-render
  - Use useCallback() for event handlers passed as props
  - Use useMemo() for expensive computations
  - Profile component renders with React DevTools Profiler
  - Test: verify no unnecessary re-renders on message add
- [ ] T096 Implement efficient state updates:
  - Ensure state updates are batched (React 18 automatic batching)
  - Avoid creating new objects on every render (useCallback, useMemo)
  - Test: verify state updates don't trigger excessive re-renders

### Polish Tasks: Accessibility

- [ ] T097 Comprehensive accessibility audit:
  - Run axe DevTools on all chat widget screens
  - Fix any violations (contrast, ARIA, keyboard nav)
  - Test with screen reader (NVDA, JAWS, VoiceOver)
  - Verify keyboard-only navigation (no mouse needed)
  - Test focus management (Tab through all interactive elements)
  - Verify WCAG AA compliance
- [ ] T098 Add ARIA landmarks:
  - Ensure ChatModal has role="dialog" and aria-modal="true"
  - Add aria-label to interactive elements
  - Add aria-describedby for help text
  - Use aria-live for dynamic message updates
  - Add aria-busy during loading
- [ ] T099 Implement focus management:
  - Focus moves to modal when opened (focus trap)
  - Focus returns to icon when modal closed
  - Focus trap prevents Tab outside modal
  - Test: Tab through modal, verify trap works
- [ ] T100 Test with assistive technologies:
  - Screen reader test (NVDA, VoiceOver, TalkBack)
  - Voice control test (if available)
  - Zoom test (up to 200%)
  - High contrast mode test
  - Dyslexia-friendly font option (optional)

### Polish Tasks: Testing Coverage

- [ ] T101 Achieve ≥ 80% unit test coverage:
  - Run jest with --coverage flag
  - Identify uncovered lines
  - Add tests for edge cases
  - Target: ≥ 80% line coverage, ≥ 75% branch coverage
- [ ] T102 Create test coverage report:
  - Generate coverage report: npm run test:coverage
  - Commit coverage report to repo
  - Monitor for coverage regressions
- [ ] T103 Add integration tests for hooks:
  - Test useChat() with real API calls (mocked)
  - Test useSelectedText() with real DOM
  - Test useConversationPersistence() with localStorage
  - Test error scenarios (network error, timeout)
- [ ] T104 Create visual regression tests (optional):
  - Use Percy or similar for screenshot comparison
  - Test all component states (loading, error, success)
  - Test mobile and desktop views
  - Detect unexpected style changes

### Polish Tasks: Browser Compatibility & Testing

- [ ] T105 Test on multiple browsers:
  - Chrome/Chromium (latest)
  - Firefox (latest)
  - Safari (latest)
  - Edge (latest)
  - Mobile browsers: Safari iOS, Chrome Android
  - Verify layout, functionality, styling
- [ ] T106 Test on multiple OS:
  - Windows (10, 11)
  - macOS (12, 13, 14)
  - iOS (14, 15, 16, 17)
  - Android (11, 12, 13, 14)
  - Test accessibility features per OS
- [ ] T107 [P] Create browser compatibility test file `src/__tests__/compatibility.test.ts`:
  - Test feature detection (localStorage, fetch, crypto, etc.)
  - Graceful degradation if features unavailable
  - Polyfill check if needed
  - Test on older browser versions if needed

### Polish Tasks: Security

- [ ] T108 Security audit for XSS vulnerabilities:
  - Ensure user input is sanitized before display
  - Use React's automatic escaping (default)
  - Verify no innerHTML usage with user input
  - Test: inject <script> in message, verify it's escaped
  - Test: inject HTML in selected text, verify it's escaped
- [ ] T109 Security audit for CSRF:
  - Verify API calls include CSRF token if needed
  - Check CORS headers (backend should validate origin)
  - Test on different origin (cross-domain request)
- [ ] T110 Audit environment variables:
  - Ensure no secrets in frontend code
  - Backend URL is public (okay to expose)
  - API keys should NOT be in frontend (backend should handle)
  - Review .env.local template (no secrets)
- [ ] T111 Add security headers check:
  - Verify CSP (Content Security Policy) allows necessary resources
  - Test: inject external script, verify CSP blocks it
  - Check X-Frame-Options if needed

### Polish Tasks: Deployment & Build

- [ ] T112 Create production build configuration:
  - npm run build creates optimized bundle
  - All assets minified and gzipped
  - Source maps available for debugging (without source code)
  - Bundle < 50KB (minified + gzipped)
- [ ] T113 Create .github/workflows/chat-widget-ci.yml for CI/CD:
  - Run lint on push: eslint src/components/ChatWidget/
  - Run tests on push: npm test
  - Run E2E tests on push (optional, slow): npm run test:e2e
  - Check bundle size: npm run build
  - Publish coverage report
- [ ] T114 Create deployment checklist in `DEPLOYMENT.md`:
  - Pre-deployment: run all tests, build succeeds, coverage ≥ 80%
  - Deployment: merge to main, CI/CD runs
  - Post-deployment: verify on production, no console errors
  - Rollback plan if issues
  - Monitoring: check error rates, user feedback
- [ ] T115 Create release notes template in `RELEASE.md`:
  - Features added
  - Bugs fixed
  - Breaking changes (if any)
  - Upgrade instructions
  - Known issues
  - Next planned features

### Polish Tasks: Monitoring & Debugging

- [ ] T116 Implement error logging:
  - Log all API errors to console and (optionally) error tracking service
  - Include request ID, timestamp, error message, stack trace
  - Test: trigger error, verify logging
- [ ] T117 Implement debug mode:
  - Enable via environment variable: DEBUG_CHAT_WIDGET=true
  - Log all state changes
  - Log all API calls and responses
  - Log user interactions (click, type, send)
  - Test: enable debug mode, verify console logs
- [ ] T118 Create performance profiling guide in `PERFORMANCE.md`:
  - How to measure bundle size
  - How to profile component renders
  - How to detect memory leaks
  - Common performance pitfalls
  - Optimization tips

### Polish Tasks: Final Quality Assurance

- [ ] T119 Create comprehensive testing checklist `TESTING_CHECKLIST.md`:
  - ✅ Unit tests: all components, hooks, utilities
  - ✅ Integration tests: component interactions, API calls
  - ✅ E2E tests: full user flows
  - ✅ Mobile testing: responsive design, touch interactions
  - ✅ Accessibility testing: WCAG AA compliance
  - ✅ Performance testing: load time, bundle size, animations
  - ✅ Browser testing: multiple browsers and OS
  - ✅ Security testing: XSS, CSRF, input validation
  - ✅ Dogfooding: real user testing on actual site
- [ ] T120 Dogfooding on production site:
  - Deploy to staging/preview
  - Internal team tests: ask actual questions, get responses
  - Verify answers are accurate and helpful
  - Collect feedback: UI/UX, performance, bugs
  - Fix any issues found
  - Deploy to production
- [ ] T121 Create post-launch monitoring plan:
  - Monitor error rates
  - Monitor API latency
  - Monitor user engagement (clicks, messages sent)
  - Monitor feedback (ratings, comments)
  - Create alerts: error rate > 5%, API latency > 5s
  - Weekly review of metrics
- [ ] T122 Create feature request & bug report template:
  - GitHub issue templates for bugs and features
  - Bug report: steps to reproduce, expected vs actual, browser/OS
  - Feature request: user story, acceptance criteria, priority
  - Link to existing documentation (quickstart, API docs)

---

## Summary Statistics

**Total Tasks**: 122 tasks  
**Parallelizable Tasks**: 35+ tasks marked with [P]  
**User Story Breakdown**:
- Phase 1 Setup: 14 tasks
- Phase 2 Foundational: 13 tasks
- Phase 3 (US1): 39 tasks
- Phase 4 (US2): 10 tasks
- Phase 5 (US3): 13 tasks
- Phase 6 (US4): 19 tasks
- Phase 7 Polish: 14 tasks

**MVP Scope** (Core Chat, ~30 tasks):
- Phase 1: All 14 tasks
- Phase 2: All 13 tasks
- Phase 3: Core tasks only (T028-T048, ~21 tasks)
- **Total MVP: ~48 tasks**

**First Release** (MVP + Mobile, ~55 tasks):
- Phase 1: All 14 tasks
- Phase 2: All 13 tasks
- Phase 3: All 39 tasks
- Phase 5: Tasks T059-T071 (mobile optimization, ~13 tasks)
- **Total First Release: ~79 tasks**

**Full Feature** (All phases, ~122 tasks):
- All 122 tasks across all 7 phases

---

## Parallel Execution Example: Phase 3 (User Story 1)

After Phase 2 completes, these tasks can run in parallel:

**Batch 1** (Component styling):
- T033: ChatIcon.module.css
- T034: ChatModal.module.css
- T035: MessageList.module.css
- T036: Message.module.css
- T037: MessageInput.module.css
- T039: ChatWidget.module.css

**Batch 2** (Component implementation):
- T028: ChatIcon.tsx
- T029: ChatModal.tsx
- T030: MessageList.tsx
- T031: Message.tsx
- T032: MessageInput.tsx

**Batch 3** (Testing):
- T042: ChatIcon tests
- T043: ChatModal tests
- T044: MessageList tests
- T045: Message tests
- T046: MessageInput tests

**Sequential** (Integration & E2E):
- T038: ChatWidget.tsx (depends on all components)
- T040: Root.tsx integration (depends on ChatWidget)
- T047: Integration test
- T048: E2E test

**Estimated Timeline**:
- Batch 1: 4 hours (parallel)
- Batch 2: 6 hours (parallel)
- Batch 3: 4 hours (parallel)
- Sequential: 6 hours
- **Total Phase 3: ~16 hours** (parallelized vs ~24 hours sequential)

---

## Next Steps

1. **Review Tasks**: Stakeholders review task list for accuracy and priorities
2. **Start Phase 1**: Initialize project structure and setup (Day 1, ~4 hours)
3. **Start Phase 2**: Implement foundational infrastructure (Day 1-2, ~6 hours)
4. **Start Phase 3**: Implement core chat functionality (Day 2-3, ~16 hours)
5. **Dogfooding**: Test on actual Docusaurus site, collect feedback (Day 4)
6. **Phases 4-7**: Complete selected text, mobile, persistence, polish (Day 5+)

---

## Acceptance Criteria

✅ All tasks follow the checklist format (checkbox, ID, labels, file paths)  
✅ Tasks organized by user story with clear dependencies  
✅ Each task is independently testable and completable  
✅ File paths are specific and actionable  
✅ Parallel opportunities identified (35+ [P] tasks)  
✅ MVP scope clearly defined (~48 tasks)  
✅ Full feature scope included (122 total tasks)  
✅ Testing strategy defined (unit, integration, E2E, accessibility)  
✅ Success criteria measurable (bundle size, performance, coverage)  
✅ Risk mitigation included (error handling, security, browser compatibility)
