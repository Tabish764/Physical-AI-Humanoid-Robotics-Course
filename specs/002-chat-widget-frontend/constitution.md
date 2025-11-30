<!-- Sync Impact Report (v1.0.0) -->
<!-- Version Change: N/A (new constitution) -->
<!-- Added Sections: Core Principles (6), UI/UX Standards, Backend Integration, Accessibility Standards, Mobile Responsiveness -->
<!-- Templates Updated: None (new feature) -->

# Chat Widget Frontend Constitution

## Core Principles

### I. Non-Intrusive Floating Widget Design
The chat widget MUST be a minimal, ChatGPT-style floating component that seamlessly integrates with Docusaurus without disrupting existing site functionality. The widget MUST NOT break the book's layout, navigation, sidebar, navbar, or any existing Docusaurus features. All chat functionality is optional and enhancement-only.

### II. Seamless Docusaurus Integration
The widget MUST work with Docusaurus 2.x and 3.x without modifying core Docusaurus files (except for theme swizzling via `src/theme/Root.tsx`). The widget MUST persist across page navigation, maintain state through route changes, and render on all pages without performance degradation.

### III. Clean, Modern UI Matching Theme
The chat widget MUST visually align with the Docusaurus theme using CSS variables (e.g., `--ifm-color-primary`) and MUST NOT hardcode colors or styles that conflict with existing Docusaurus components. Styling MUST use CSS modules or scoped styles to prevent CSS leakage into the document.

### IV. Direct Backend Communication with Proper Error Handling
The widget MUST communicate directly with the FastAPI backend via REST API (`POST /api/chat`, `POST /api/chat-selected`, display sources). All API calls MUST include timeout handling (30-second maximum), graceful error messages, and retry logic. Network failures MUST NOT crash the widget.

### V. Responsive Design: Mobile-First Approach
The widget MUST work flawlessly on mobile devices (< 768px width) with full-screen modal, touch-friendly buttons (≥44px tap targets), auto-showing keyboard on input focus, and responsive chat icon sizing (50px on mobile, 60px on desktop). Desktop responsiveness (400px width × 600px height) MUST be optimized for readability.

### VI. React 18+ with TypeScript and Functional Components
The widget MUST be built using React 18+ functional components with hooks (useState, useEffect, useContext). TypeScript MUST be used for type safety, with all functions and components properly typed. No class components or legacy patterns allowed.

## UI/UX Standards

### Chat Icon Specifications
- **Position**: Fixed bottom-right corner (20px from edges on desktop, responsive on mobile)
- **Size**: 60px × 60px circular button (50px on mobile)
- **Icon**: Chat bubble or message icon (SVG recommended)
- **Color**: Primary theme color or #1976d2 (blue)
- **Hover Effect**: Scale animation (1.05x) with smooth transition (300ms ease)
- **Z-index**: 9999 (always on top, above all content)
- **Accessibility**: ARIA label ("Open chat assistant"), keyboard accessible (Tab + Enter)

### Chat Modal Specifications
- **Position**: Fixed bottom-right, above chat icon (respects viewport)
- **Desktop Size**: 400px width × 600px height
- **Mobile Size**: Full screen or 90% viewport width/height
- **Header**: "AI Assistant" title + X close button
- **Border-radius**: 12px for modern appearance
- **Shadow**: `box-shadow: 0 4px 12px rgba(0,0,0,0.15)` for depth
- **Animation**: Slide-up (200ms) + fade-in (300ms ease)
- **Close Methods**: X button, Escape key, outside click (optional)

### Message Display Format
- **User Messages**: Right-aligned, blue background (#1976d2 or theme primary), white text, rounded corners (8px)
- **AI Messages**: Left-aligned, light gray background (#f5f5f5), dark text, rounded corners (8px)
- **Timestamps**: Small gray text (12px) below each message (HH:MM format)
- **Auto-scroll**: Chat MUST auto-scroll to newest message
- **Message Area**: Scrollable with 16px padding, max-height respects modal height

### Input Area Design
- **Layout**: Text input field + send button at bottom of modal (sticky)
- **Input Field**: Full width minus button, 12px padding, border (1px solid #e0e0e0), focus state (border color to primary)
- **Send Button**: 40px × 40px, disabled during loading (opacity 0.5), shows loading spinner
- **Keyboard**: Enter to send, Shift+Enter for new line (optional), Escape to close modal
- **Auto-focus**: Input field receives focus when modal opens

### Source Display
- **Format**: Clickable links below AI responses (blue, underlined)
- **Text**: "Sources: [file1.md](link) • [file2.md](link)"
- **Click Behavior**: Navigate to book page or open in new tab
- **Styling**: Match Docusaurus link colors, hover effect (darker blue)

## Backend Integration Standards

### API Communication Protocol
- **Base URL**: Configurable via environment variable (REACT_APP_BACKEND_URL) or Docusaurus customFields
- **Endpoints**:
  - `POST /api/chat` with payload `{"question": "user input"}` → response `{"answer": "...", "sources": [...]}`
  - `POST /api/chat-selected` with payload `{"question": "...", "selected_text": "..."}` → response `{"answer": "...", "sources": []}`
- **Headers**: `Content-Type: application/json`, CORS headers must be set by backend
- **Timeout**: 30-second maximum timeout per request
- **Loading State**: Show spinner during API call (not longer than 5 seconds in normal conditions)

### Error Handling Requirements
- **Network Error**: Display "Check your internet connection. Please try again."
- **Timeout Error**: Display "Request took too long. Please try again." (after 30 seconds)
- **API Error**: Display "Failed to connect to AI. Please try again." with optional retry button
- **Invalid Response**: Log error, display generic error message, do NOT crash widget
- **Backend Down**: Gracefully handle 5xx errors with user-friendly message
- **Rate Limit (503)**: Display "Service busy. Please try again in a moment."

### Data Validation
- **User Input**: Trim whitespace, min 1 char, max 2000 chars per message
- **API Response**: Validate answer (string, non-empty), sources (array of strings), handle missing fields
- **Selected Text**: Validate selected_text (min 1 char, max 5000 chars)

## Accessibility Standards

### ARIA and Semantic HTML
- **Chat Icon Button**: `<button aria-label="Open AI assistant chat">`, role="button"
- **Modal**: `role="dialog"`, `aria-modal="true"`, `aria-labelledby="chat-header"`
- **Message List**: `role="log"`, `aria-live="polite"` (announce new messages)
- **Input Field**: `aria-label="Chat message input"`
- **Send Button**: `aria-label="Send message"`
- **Close Button**: `aria-label="Close chat"`

### Keyboard Navigation
- **Tab**: Navigate through all interactive elements (icon, input, send button, close button)
- **Enter**: Send message (when input focused), open modal (when icon focused)
- **Escape**: Close modal (when modal open)
- **Focus Management**: Trap focus inside modal (Tab cycles through modal elements only)
- **Focus Indicator**: Visible focus ring on all buttons (4px outline)

### Color Contrast
- **WCAG AA Compliance**: All text MUST have contrast ratio ≥ 4.5:1 for normal text, ≥ 3:1 for large text
- **Text on Blue (#1976d2)**: White text (contrast 8.59:1 ✓)
- **Text on Gray (#f5f5f5)**: Dark gray text (contrast 12:1 ✓)

### Screen Reader Support
- **Live Regions**: New messages announced via `aria-live="polite"`
- **Message Context**: Each message includes role and timestamp for clarity
- **Error Messages**: Announced immediately, no visual-only errors
- **Loading State**: "Loading..." message announced when spinner appears

## Mobile Responsiveness Standards

### Breakpoints and Behavior
- **< 768px (Mobile)**:
  - Modal takes full screen (100vh, 100vw with safe area inset)
  - Chat icon: 50px × 50px
  - Input field: 12px padding (touch-friendly)
  - Send button: 44px × 44px minimum
  - Font size: 16px (prevents zoom on focus)

- **≥ 768px (Desktop)**:
  - Modal: 400px × 600px fixed size
  - Chat icon: 60px × 60px
  - Standard padding and spacing

### Touch Interaction
- **Button Tap Targets**: Minimum 44px × 44px (mobile)
- **Touch Scrolling**: Chat message list scrolls smoothly (momentum scroll)
- **Keyboard**: Auto-shows when input focused
- **Haptic Feedback**: Optional vibration on send (navigator.vibrate)
- **Swipe Gestures**: Swipe down to close modal (optional enhancement)

### Performance on Mobile
- **Bundle Size**: Chat widget JS ≤ 50KB (minified + gzipped)
- **Initial Load**: < 100ms to interactive
- **No Jank**: Smooth 60fps animations (transform, opacity only)
- **Battery Efficient**: No long polling, use short timeouts, cleanup on unmount

## State Management and Component Architecture

### React Component Structure
```
ChatWidget (main container)
├── ChatIcon (floating button)
└── ChatModal (when open)
    ├── ChatHeader (title + close)
    ├── MessageList (chat history)
    │   └── Message (individual message)
    └── ChatInput (text input + send button)
    └── LoadingSpinner (during API call)
```

### State Shape (ChatWidget)
```typescript
{
  messages: Array<{
    role: 'user' | 'assistant',
    content: string,
    timestamp: Date,
    sources?: string[]
  }>,
  isLoading: boolean,
  error: string | null,
  isOpen: boolean,
  inputValue: string
}
```

### Key Behaviors
- **Message Clear**: Clear input field immediately after sending (optimistic UI)
- **Message Persistence**: Chat history persists during session (localStorage optional for future)
- **Auto-scroll**: Scroll to bottom on new message (useEffect)
- **Focus Management**: Auto-focus input when modal opens
- **Cleanup**: Cancel API requests on unmount (useEffect cleanup)

## Docusaurus Integration

### Theme Swizzling
- **File**: `src/theme/Root.tsx` (Docusaurus theme wrapper)
- **Content**: Import and render `<ChatWidget />` at root level
- **No Modifications**: Do NOT modify core Docusaurus files (layout.tsx, navbar, sidebar)
- **Persistence**: ChatWidget persists across page navigation (maintained in Root context)

### Environment Configuration
- **Backend URL**: Read from `.env.local` (`REACT_APP_BACKEND_URL`) or `docusaurus.config.js` customFields
- **Fallback**: Default to `http://localhost:8000` (development)
- **Access**: Use `useDocusaurusContext()` hook to read configuration

### CSS Isolation
- **CSS Modules**: `ChatWidget.module.css` with BEM naming convention
- **No Global Styles**: All styles scoped to `ChatWidget` and sub-components
- **CSS Variables**: Use Docusaurus theme variables (--ifm-color-primary, --ifm-color-background, etc.)
- **Override Precedence**: CSS modules MUST NOT conflict with Docusaurus theme

## Development Workflow and Quality Gates

### Code Quality Standards
- **TypeScript**: Strict mode enabled, no `any` types
- **Linting**: ESLint with React/TypeScript rules, Prettier for formatting
- **Testing**: Unit tests for components (React Testing Library), integration tests for API calls
- **No Console Errors**: Zero warnings/errors in browser console
- **No Memory Leaks**: All async operations cleaned up on unmount

### Testing Requirements
- **Unit Tests**: ChatWidget, ChatIcon, ChatModal, ChatInput components
- **Integration Tests**: API calls, error handling, message display
- **E2E Tests**: User flow (open modal → type message → send → display answer → close)
- **Mobile Tests**: Responsive behavior on < 768px viewports
- **Accessibility Tests**: ARIA labels, keyboard nav, focus management

### Performance Benchmarks
- **Initial Load**: ChatWidget renders in < 100ms
- **Modal Open**: Animation completes in 300ms
- **API Call**: Response displayed within 5 seconds (normal)
- **Bundle Size**: < 50KB minified + gzipped
- **Memory**: No memory leaks on long sessions (repeat send/close cycles)

### Deployment Checklist
- ✅ No hardcoded secrets or API keys
- ✅ Environment variables properly configured
- ✅ CORS headers correct on backend
- ✅ All TypeScript types strict
- ✅ CSS scoped, no conflicts
- ✅ Accessibility audit passed (WCAG AA)
- ✅ Mobile testing on real devices
- ✅ Error messages user-friendly
- ✅ Loading states visible
- ✅ Keyboard navigation works
- ✅ No console errors
- ✅ Builds without warnings

## Constraints and Non-Goals

### Hard Constraints
- MUST work with Docusaurus 2.x and 3.x
- MUST NOT use jQuery or heavy dependencies (React hooks only)
- MUST NOT break existing Docusaurus functionality
- MUST be configurable (backend URL, not hardcoded)
- MUST NOT exceed 50KB bundle size
- MUST be accessible (WCAG AA minimum)
- MUST handle errors gracefully (no white-screen crashes)

### Non-Goals (MVP)
- User authentication UI (backend handles auth)
- Voice input/output
- File upload capability
- Multi-language UI (English only)
- Conversation history sync across devices
- Real-time typing indicators
- Emoji picker
- Message editing/deletion
- Dark mode toggle (use Docusaurus theme)

## Governance

### Amendment Procedure
1. Changes to principles require architectural decision record (ADR)
2. Version bumps follow semantic versioning:
   - **MAJOR**: Principle removals or incompatible changes (e.g., remove accessibility requirement)
   - **MINOR**: New principle or major expansion (e.g., add selected text feature)
   - **PATCH**: Clarifications, wording, non-semantic refinements
3. All PRs MUST verify compliance with constitution
4. Code review MUST check against accessibility and performance standards

### Compliance Review
- **Monthly**: Review open issues/PRs for constitution violations
- **On Release**: Full checklist audit (development workflow section)
- **On Architecture Change**: Re-validate integration constraints

---

**Version**: 1.0.0 | **Ratified**: 2025-11-29 | **Last Amended**: 2025-11-29
