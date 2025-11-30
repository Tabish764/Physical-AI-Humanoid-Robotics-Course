# Implementation Plan: Chat Widget Frontend for Docusaurus

**Branch**: `002-chat-widget-frontend` | **Date**: 2025-11-29 | **Spec**: [Chat Widget Specification](spec.md)
**Input**: Feature specification from `/specs/002-chat-widget-frontend/spec.md`

## Summary

Build a floating, ChatGPT-style chat widget that seamlessly integrates with the Docusaurus Physical AI textbook website. Users can ask questions about book content and receive AI-generated answers with source citations from the FastAPI RAG backend. The widget must be non-intrusive, responsive (mobile-first), accessible, and persist conversation history across page navigation. Key features include: floating chat icon (bottom-right), modal-based chat interface, support for explaining selected text, error handling, and mobile-optimized UI. Integration via Docusaurus theme swizzling (`src/theme/Root.tsx`) ensures global availability on all pages without breaking existing functionality.

## Technical Context

**Language/Version**: JavaScript (ES2020+) or TypeScript 4.9+, React 18+  
**Primary Dependencies**: React 18+, React Hooks (useState, useEffect, useContext, useCallback), Docusaurus 2.x or 3.x  
**Storage**: Session state only (React state, localStorage optional for future persistence)  
**Testing**: Jest + React Testing Library (unit/integration tests), Playwright (E2E tests)  
**Target Platform**: Web browser (desktop and mobile), responsive design < 768px breakpoint  
**Project Type**: Frontend component (Docusaurus theme integration)  
**Performance Goals**: 
- Initial load: < 100ms to interactive
- Modal open/close animation: 200-300ms
- API response display: < 5 seconds (normal conditions)
- Bundle size: ≤ 50KB (minified + gzipped)

**Constraints**: 
- Must work with Docusaurus 2.x and 3.x without version-specific hacks
- No jQuery or heavy UI dependencies (React hooks only)
- CSS must not conflict with Docusaurus theme (CSS modules required)
- Must maintain all existing Docusaurus functionality (navbar, sidebar, content navigation)
- Backend URL must be configurable (environment variable or docusaurus.config.js)
- Session-based state (no cross-session persistence for MVP)

**Scale/Scope**: 
- Single Docusaurus site (50-100 pages estimated)
- Up to 10-20 concurrent users (MVP scale)
- Conversation history per session (100+ messages feasible)
- API timeout: 30 seconds per request
- Mobile-first responsive design (< 768px full-screen modal)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Core Principles (from constitution):

✅ **I. Non-Intrusive Floating Widget Design**
- Floating icon in bottom-right corner (z-index: 9999)
- Modal opens on click, no disruption to existing layout
- Chat widget optional and enhancement-only
- Does not break existing Docusaurus functionality

✅ **II. Seamless Docusaurus Integration**
- Integration via `src/theme/Root.tsx` (Docusaurus theme swizzling)
- Works with Docusaurus 2.x and 3.x
- Persists across page navigation (component mounted at root level)
- No modifications to core Docusaurus files required

✅ **III. Clean, Modern UI Matching Theme**
- Uses CSS variables (--ifm-color-primary, etc.)
- CSS modules to prevent style leakage
- Visually aligns with Docusaurus design
- No hardcoded colors or styles that conflict

✅ **IV. Direct Backend Communication with Proper Error Handling**
- REST API: POST /api/chat, POST /api/chat-selected
- 30-second timeout per request
- Graceful error messages (network error, timeout, server error)
- Retry logic for failed requests
- No widget crash on API failures

✅ **V. Responsive Design: Mobile-First Approach**
- Desktop: 400px × 600px modal, 60px × 60px icon
- Mobile (< 768px): Full-screen modal, 50px × 50px icon
- Touch-friendly buttons (≥44px tap targets)
- Auto-showing keyboard on input focus
- Smooth scrolling and no jank on mobile

✅ **VI. React 18+ with TypeScript and Functional Components**
- React 18+ functional components with hooks (useState, useEffect, useCallback)
- TypeScript for type safety
- No class components
- Proper cleanup on unmount (useEffect dependencies)

### Key Standards (Invariants):

✅ **Code Quality**
- TypeScript strict mode, no `any` types
- All functions and components properly typed
- ESLint + Prettier for consistent formatting

✅ **Accessibility**
- ARIA labels on all interactive elements
- Keyboard navigation (Tab, Enter, Escape)
- Focus management (trap focus in modal)
- WCAG AA color contrast (≥ 4.5:1 for normal text)
- Screen reader announcements for new messages

✅ **Error Handling**
- User-friendly error messages
- No console errors or warnings
- Graceful degradation on API failures
- Retry button for failed requests

✅ **Performance**
- < 100ms initial load
- ≤ 50KB bundle size (minified + gzipped)
- 60fps animations (transform, opacity only)
- No memory leaks on long sessions
- Proper cleanup on unmount

✅ **Testing**
- Unit tests for components
- Integration tests for API calls
- E2E tests for user flows
- Mobile responsiveness testing
- Accessibility audit (WCAG AA)

---

## Project Structure

### Documentation (this feature)

```
specs/002-chat-widget-frontend/
├── constitution.md       # Feature-level constitution
├── spec.md              # Feature specification (this document references it)
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (research findings)
├── data-model.md        # Phase 1 output (entity definitions)
├── quickstart.md        # Phase 1 output (setup and run instructions)
├── contracts/           # Phase 1 output (API contracts)
│   └── api-contracts.md
├── checklists/
│   └── requirements.md   # Quality checklist (already created)
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

```
src/
├── components/
│   └── ChatWidget/
│       ├── ChatWidget.tsx           # Main container component
│       ├── ChatWidget.module.css    # Scoped styles
│       ├── ChatIcon.tsx             # Floating button component
│       ├── ChatModal.tsx            # Modal wrapper component
│       ├── MessageList.tsx          # Chat history display
│       ├── Message.tsx              # Individual message component
│       ├── MessageInput.tsx         # Text input + send button
│       ├── LoadingSpinner.tsx       # Loading indicator
│       ├── SelectedTextButton.tsx   # "Ask AI" for selected text
│       ├── types.ts                 # TypeScript type definitions
│       ├── useChat.ts               # Custom hook for chat logic
│       ├── useSelectedText.ts       # Custom hook for text selection
│       └── api.ts                   # API client functions
└── theme/
    └── Root.tsx                     # Docusaurus theme wrapper (integrates ChatWidget)

tests/
├── unit/
│   ├── ChatWidget.test.tsx
│   ├── ChatIcon.test.tsx
│   ├── ChatModal.test.tsx
│   ├── MessageList.test.tsx
│   └── Message.test.tsx
├── integration/
│   ├── chat-api.test.tsx
│   ├── message-sending.test.tsx
│   └── error-handling.test.tsx
└── e2e/
    ├── user-chat-flow.spec.ts
    ├── mobile-responsiveness.spec.ts
    └── accessibility.spec.ts

.env.example
docusaurus.config.js (REACT_APP_BACKEND_URL configuration)
package.json (dependencies: react 18+, typescript, jest, testing-library)
```

**Structure Decision**: Web application (frontend-only for this feature, integrates with existing Docusaurus site). Single React component with sub-components organized by responsibility. Custom hooks for business logic (chat, text selection). API layer separated for testability. Tests organized by type (unit, integration, E2E). CSS modules prevent style conflicts.

---

## Complexity Tracking

No constitution violations detected. All core principles and standards directly supported by the technical context and planned implementation.

---

## Key Decisions & Rationale

### 1. React Hooks + Functional Components
**Decision**: Use React 18+ functional components with hooks (useState, useEffect, useCallback, useContext)
**Rationale**: Modern React best practice, easier testing, better performance (no class overhead), aligns with Docusaurus patterns
**Alternatives Considered**:
- Class components: Rejected (legacy, harder to test, more boilerplate)
- Vue/Svelte: Rejected (incompatible with Docusaurus React ecosystem)

### 2. CSS Modules for Styling
**Decision**: Use CSS modules (ChatWidget.module.css) with BEM naming convention
**Rationale**: Prevents style conflicts with Docusaurus theme, scoped to component, no CSS-in-JS overhead
**Alternatives Considered**:
- Tailwind CSS: Rejected (could conflict with Docusaurus utilities)
- CSS-in-JS (styled-components): Rejected (adds bundle size, harder to scope)
- Docusaurus theme customization: Rejected (requires modifying core files)

### 3. Session-Based State (React State Only)
**Decision**: Use React state for conversation history (localStorage optional for future)
**Rationale**: MVP simplicity, no backend persistence needed, state resets on page reload (acceptable for MVP)
**Alternatives Considered**:
- localStorage: Rejected for MVP (scope creep, adds complexity)
- Backend persistence: Rejected (requires user IDs, auth, increases scope)

### 4. Custom Hooks for Logic Separation
**Decision**: Extract chat logic (useChat.ts) and text selection logic (useSelectedText.ts) into custom hooks
**Rationale**: Testable, reusable, cleaner component hierarchy, easier debugging
**Alternatives Considered**:
- Logic in components: Rejected (harder to test, less reusable)
- Redux/Context API: Rejected (overkill for MVP, adds bundle size)

### 5. Docusaurus Theme Swizzling via Root.tsx
**Decision**: Integrate ChatWidget globally via `src/theme/Root.tsx` (Docusaurus theme wrapper)
**Rationale**: Renders on all pages, persists across navigation, no modifications to core Docusaurus files
**Alternatives Considered**:
- Swizzle Layout.tsx: Rejected (too intrusive, high breaking change risk)
- Client-side script injection: Rejected (fragile, hard to maintain)
- Docusaurus plugin: Rejected (overly complex for this feature)

### 6. REST API (not GraphQL)
**Decision**: Use REST API (POST /api/chat, POST /api/chat-selected)
**Rationale**: Matches existing backend design, simple request/response, no additional complexity
**Alternatives Considered**:
- GraphQL: Rejected (backend is REST, adds client-side complexity)
- WebSockets: Rejected (streaming not required, adds latency tracking complexity)

### 7. TypeScript (Strict Mode)
**Decision**: Use TypeScript with strict mode enabled (no `any` types)
**Rationale**: Type safety reduces bugs, better IDE support, documents component contracts
**Alternatives Considered**:
- JavaScript: Rejected (loses type safety benefits, harder to maintain)
- Loose TypeScript: Rejected (defeats purpose, allows `any` casts)

---

## Next Steps (Phase 0 & Phase 1)

### Phase 0: Research
1. Investigate Docusaurus 2.x vs 3.x theme swizzling differences
2. Research React 18 concurrent features and performance optimization
3. Evaluate testing strategies (Jest vs Vitest, Testing Library patterns)
4. Determine CSS module configuration for Docusaurus build
5. Research mobile touch event handling and keyboard handling patterns

### Phase 1: Design & Contracts
1. Create `data-model.md` (Chat Message, Conversation State entities)
2. Create `contracts/api-contracts.md` (OpenAPI spec for /api/chat, /api/chat-selected)
3. Create `quickstart.md` (setup, build, run instructions)
4. Create `research.md` (consolidate Phase 0 findings)
5. Update agent context with React 18 + TypeScript specifics

---

**Version**: 1.0.0 | **Status**: Ready for Phase 0 Research

