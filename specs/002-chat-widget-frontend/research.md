# Research Findings: Chat Widget Frontend

**Date**: 2025-11-29
**Feature**: Interactive AI Chat Widget for Docusaurus Book
**Scope**: Technology evaluation, best practices, integration patterns

## Docusaurus Theme Integration

### Decision: Use `src/theme/Root.tsx` (Theme Swizzling)

**Rationale**: 
Docusaurus provides theme swizzling as the recommended way to customize rendering across all pages. The `Root.tsx` component wraps the entire site and is rendered on every page, making it the ideal place to mount the ChatWidget globally.

**Selected Approach**: 
- Create `src/theme/Root.tsx` that wraps the original Docusaurus Root
- Import and render `<ChatWidget />` inside Root
- ChatWidget maintains state and persists across page navigation
- No modifications to core Docusaurus Layout or other components required

**Alternatives Considered**:
1. **Swizzle Layout.tsx**: More intrusive, higher risk of breaking Docusaurus layout logic
2. **Client-side script injection**: Fragile, hard to maintain, security concerns
3. **Docusaurus plugin with clientModule**: Overly complex, requires plugin infrastructure

**Why Root.tsx is superior**: 
- Wraps all content, executes first in render tree
- Persists across route changes (React Router handles navigation)
- Clean separation of concerns (ChatWidget doesn't need to know about Docusaurus internals)
- Easy to enable/disable by commenting out ChatWidget component
- Aligns with Docusaurus official documentation patterns

---

## React 18 Implementation Strategy

### Decision: Functional Components with Hooks (useState, useEffect, useCallback, useContext)

**Rationale**:
React 18 introduced concurrent rendering and automatic batching, improving performance. Functional components with hooks are the modern standard and offer better performance characteristics than class components.

**Selected Approach**:
- Build all components as functional components
- Use `useState` for local component state (messages, loading, error, inputValue)
- Use `useEffect` for side effects (API calls, focus management, cleanup)
- Use `useCallback` for memoized event handlers (prevent unnecessary re-renders)
- Use `useContext` for optional theme/config sharing (if needed)
- Custom hooks (useChat, useSelectedText) for business logic extraction

**Performance Optimizations**:
1. **Debounce rapid submissions**: useCallback with ref to track last submission time
2. **Lazy re-renders**: React.memo on Message components to prevent re-rendering on parent updates
3. **useCallback dependencies**: Minimize dependency arrays to prevent function recreation
4. **useEffect cleanup**: Proper cleanup to prevent memory leaks (cancel async operations on unmount)

**Code Structure**:
```
ChatWidget.tsx (main component, manages state)
  ├── useChat hook (chat API logic)
  ├── useSelectedText hook (text selection logic)
  ├── ChatIcon component (memoized)
  ├── ChatModal component
  │   ├── MessageList component
  │   │   └── Message component (memoized, with React.memo)
  │   ├── MessageInput component
  │   └── LoadingSpinner component
  └── SelectedTextButton component
```

**Why this structure**:
- Separation of concerns: components handle UI, hooks handle logic
- Easy to test: hooks can be tested independently with @testing-library/react
- Reusable: hooks can be extracted to other components if needed
- Performance: memoization at leaf level (Message) prevents unnecessary re-renders

---

## TypeScript Configuration

### Decision: Strict Mode, No `any` Types

**Rationale**:
TypeScript strict mode catches more errors at compile time, improves IDE support, and documents component contracts. Disallowing `any` forces explicit typing, preventing accidental type escapes.

**Selected Configuration**:
```json
{
  "compilerOptions": {
    "strict": true,
    "noImplicitAny": true,
    "strictNullChecks": true,
    "strictFunctionTypes": true,
    "noUnusedLocals": true,
    "noUnusedParameters": true,
    "noImplicitReturns": true,
    "esModuleInterop": true,
    "skipLibCheck": true,
    "forceConsistentCasingInFileNames": true
  }
}
```

**Type Definitions**:
- Create `types.ts` with explicit types for Message, Conversation, API responses
- Use `React.FC<Props>` type for components
- Export all types from types.ts for reusability

**Benefits**:
- Compiler catches null/undefined errors
- IDE provides accurate autocompletion
- Self-documenting code (types serve as documentation)
- Easier refactoring (compiler alerts on type mismatches)

---

## CSS Module Strategy

### Decision: CSS Modules with BEM Naming Convention

**Rationale**:
Docusaurus uses Infima CSS framework and has its own style system. CSS modules prevent style conflicts by scoping styles to individual components. BEM naming convention improves readability and prevents naming collisions.

**Selected Configuration**:
- `ChatWidget.module.css` contains all scoped styles
- BEM naming: `.chatWidget__icon`, `.chatWidget__modal`, `.chatWidget__message`, etc.
- CSS variables for theme colors: `var(--ifm-color-primary)`, `var(--ifm-color-background)`
- No global styles (all styles scoped)

**Example BEM Structure**:
```css
/* ChatWidget.module.css */

/* Block: chatWidget */
.chatWidget { }
.chatWidget__icon { }
.chatWidget__modal { }
.chatWidget__modal--open { }
.chatWidget__message { }
.chatWidget__message--user { }
.chatWidget__message--assistant { }
.chatWidget__input { }
.chatWidget__input:focus { }
.chatWidget__button { }
.chatWidget__button:disabled { }
```

**Why CSS Modules**:
1. Scoped to component (no global namespace pollution)
2. Docusaurus build tool supports CSS modules natively
3. Works with Docusaurus theme system (can use CSS variables)
4. No external dependencies (built-in to Webpack/Docusaurus)
5. Zero runtime overhead (compiled to vanilla CSS)

**Docusaurus Theme Variables**:
- `--ifm-color-primary`: Primary theme color (blue default)
- `--ifm-color-background`: Background color
- `--ifm-color-text`: Text color
- `--ifm-color-emphasis-800`: Hover/focus color
- `--ifm-spacing-unit`: Spacing unit (8px)

Using these variables ensures ChatWidget respects site theme changes and dark mode.

---

## Testing Strategy

### Unit Testing: Jest + React Testing Library

**Rationale**:
Jest is Docusaurus's default test runner. React Testing Library encourages testing components from a user perspective (queries, events) rather than implementation details.

**Test Coverage**:
1. **Component rendering**: ChatIcon, ChatModal, Message, MessageInput
2. **User interactions**: Click icon, type message, press Enter, click send
3. **API integration**: Mock API calls, test success/error paths
4. **State management**: Messages array, loading state, error state
5. **Edge cases**: Empty input, long input, timeout, network error

**Integration Testing**:
- Message sending and display
- Error handling and retry logic
- Text selection detection
- Modal open/close with state persistence

**E2E Testing: Playwright**:
- User flow: Open page → Click icon → Type question → Verify response
- Mobile responsiveness: Open on mobile viewport → Verify layout
- Accessibility: Keyboard navigation, ARIA labels, screen reader support

**Mock Strategy**:
- Mock `fetch` with `jest.mock('fetch')` or use `msw` (Mock Service Worker)
- Mock Docusaurus hooks: `useDocusaurusContext()`, `useColorMode()`
- Mock window.getSelection() for text selection testing

---

## Mobile Responsiveness Strategy

### Decision: Mobile-First with CSS Media Queries

**Rationale**:
Mobile-first approach ensures the widget works on smallest devices first, then progressively enhances for larger screens. Media queries handle the breakpoint at 768px (tablet).

**Breakpoints**:
1. **Mobile (< 768px)**: Full-screen modal, 50px icon, touch-friendly (44px+ buttons)
2. **Desktop (≥ 768px)**: 400×600px modal, 60px icon, standard buttons

**Mobile Optimizations**:
1. **Font size**: 16px minimum (prevents zoom on input focus)
2. **Touch targets**: 44px × 44px minimum (WCAG AA guideline)
3. **Keyboard handling**: Auto-show on input focus, don't cover input field
4. **Scrolling**: Smooth scrolling, no jank (use transform, not top/left)
5. **Viewport meta**: Ensure `<meta name="viewport">` in Docusaurus

**CSS Strategy**:
```css
/* Mobile first */
.chatWidget__modal {
  position: fixed;
  width: 100%;
  height: 100%;
  /* mobile layout */
}

/* Desktop enhancements */
@media (min-width: 768px) {
  .chatWidget__modal {
    width: 400px;
    height: 600px;
    /* desktop layout */
  }
}
```

---

## Accessibility (WCAG AA) Implementation

### Decision: ARIA Labels, Keyboard Navigation, Color Contrast

**Rationale**:
Accessibility ensures the chat widget works for all users, including those with disabilities. WCAG AA is the standard for web accessibility.

**ARIA Implementation**:
1. **Chat icon**: `aria-label="Open AI assistant chat"`
2. **Modal**: `role="dialog"`, `aria-modal="true"`, `aria-labelledby="chat-header"`
3. **Message list**: `role="log"`, `aria-live="polite"` (announce new messages)
4. **Input**: `aria-label="Chat message input"`
5. **Send button**: `aria-label="Send message"`, `aria-disabled="true"` (when loading)
6. **Close button**: `aria-label="Close chat"`

**Keyboard Navigation**:
1. **Tab**: Navigate through interactive elements
2. **Enter**: Send message, open modal
3. **Shift+Enter**: New line (optional)
4. **Escape**: Close modal
5. **Focus management**: Trap focus in modal (Tab cycles through modal elements only)
6. **Focus visible**: 4px outline on all buttons

**Color Contrast**:
1. **User messages**: White text on #1976d2 (contrast 8.59:1 ✓)
2. **AI messages**: Dark gray text on #f5f5f5 (contrast 12:1 ✓)
3. **Error messages**: Red text on white (meet WCAG AA)
4. **Links**: Blue links with underline (contrast 3:1+ ✓)

**Screen Reader Testing**:
- Announce new messages via `aria-live="polite"`
- Include message role and timestamp in announcement
- Test with NVDA (Windows), JAWS (Windows), VoiceOver (macOS/iOS)

---

## API Error Handling Strategy

### Decision: Graceful Degradation with User-Friendly Messages

**Rationale**:
Users may experience network issues, backend downtime, or rate limits. Graceful error handling keeps the widget functional and informative.

**Error Scenarios**:
1. **Network error**: `navigator.onLine === false` → "Check your internet connection"
2. **Timeout (30s)**: No response from server → "Request took too long. Please try again." + Retry button
3. **Backend error (5xx)**: `status >= 500` → "Failed to connect to AI. Please try again." + Retry button
4. **Rate limit (503)**: `status === 503` → "Service busy. Please try again in a moment."
5. **Invalid response**: Missing `answer` field → Log error, show generic message, no crash

**Implementation**:
```typescript
async function sendMessage(question: string) {
  try {
    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), 30000); // 30s timeout
    
    const response = await fetch(`${backendUrl}/api/chat`, {
      method: 'POST',
      body: JSON.stringify({ question }),
      signal: controller.signal
    });
    
    clearTimeout(timeout);
    
    if (!response.ok) {
      if (response.status === 503) {
        throw new Error('Service busy');
      } else if (response.status >= 500) {
        throw new Error('Backend error');
      }
    }
    
    const data = await response.json();
    return data; // { answer, sources }
  } catch (error) {
    if (error.name === 'AbortError') {
      setError('Request took too long');
    } else if (!navigator.onLine) {
      setError('Check your internet connection');
    } else {
      setError('Failed to connect');
    }
  }
}
```

**Retry Logic**:
- Show retry button on error
- Track failed attempts (limit to 3)
- Exponential backoff: wait 2s, 4s, 8s between retries (optional)

---

## Environment Configuration

### Decision: Environment Variables + Docusaurus Config

**Rationale**:
Backend URL should not be hardcoded. Different environments (dev, staging, prod) have different API URLs.

**Configuration Sources**:
1. **Environment variables**: `REACT_APP_BACKEND_URL` (React convention)
2. **Docusaurus customFields**: `docusaurus.config.js` → `customFields.backendUrl`
3. **Fallback**: `http://localhost:8000` (development default)

**Priority Order**:
1. `process.env.REACT_APP_BACKEND_URL` (if set)
2. `useDocausaurusContext().siteConfig.customFields.backendUrl` (from config)
3. Default `http://localhost:8000`

**Example `.env.local`**:
```
REACT_APP_BACKEND_URL=https://api.example.com
```

**Example `docusaurus.config.js`**:
```javascript
module.exports = {
  customFields: {
    backendUrl: process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000'
  }
}
```

---

## Performance Optimization

### Decision: Code Splitting, Lazy Loading, Memoization

**Rationale**:
Chat widget should have minimal impact on initial page load. Lazy loading and code splitting defer non-critical code.

**Optimizations**:
1. **Code splitting**: ChatWidget as separate chunk (dynamic import in Root.tsx)
2. **Lazy loading**: Load ChatWidget component only when page is interactive
3. **Memoization**: `React.memo()` on Message components
4. **useCallback**: Memoize event handlers to prevent unnecessary re-renders
5. **CSS-in-JS avoidance**: Use CSS modules (compiled to vanilla CSS, no runtime overhead)
6. **Bundle size target**: ≤ 50KB (minified + gzipped)

**Example Dynamic Import**:
```typescript
// Root.tsx
const ChatWidget = React.lazy(() => import('../components/ChatWidget'));

export default function Root(props) {
  return (
    <>
      <Suspense fallback={null}>
        <ChatWidget />
      </Suspense>
      {props.children}
    </>
  );
}
```

---

## Summary of Key Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Integration | `src/theme/Root.tsx` | Global rendering, persists across nav, no core mods |
| Components | React 18 + Hooks | Modern, performant, testable |
| Typing | TypeScript strict | Type safety, better IDE, self-documenting |
| Styling | CSS Modules + BEM | Scoped, theme-aware, zero runtime overhead |
| Testing | Jest + Testing Library | Docusaurus standard, user-focused testing |
| Responsive | Mobile-first CSS | Works on all devices, progressive enhancement |
| Accessibility | ARIA + Keyboard nav | WCAG AA compliant, inclusive |
| Errors | Graceful handling + user messages | No crashes, informative feedback |
| Config | Env vars + Docusaurus config | Flexible, environment-aware |
| Performance | Lazy loading + memoization | Fast initial load, optimized re-renders |

---

**Status**: Phase 0 Research Complete - Ready for Phase 1 Design

