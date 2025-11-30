# Quickstart Guide: Chat Widget Frontend

**Date**: 2025-11-29
**Feature**: Interactive AI Chat Widget for Docusaurus Book
**Environment**: Local development setup

## Prerequisites

Before you begin, ensure you have the following installed:

- **Node.js 18+** (with npm or yarn)
- **Git** (for cloning the repository)
- **Code editor** (VS Code recommended)
- **Backend API running** (FastAPI backend at `http://localhost:8000` or configured URL)

Verify your setup:
```bash
node --version  # Should be 18+ (e.g., v18.17.0)
npm --version   # Should be 9+ (e.g., 9.8.1)
```

## 1. Project Structure

The Chat Widget is integrated into the Docusaurus project:

```
ai-native-book/
├── src/
│   ├── components/
│   │   └── ChatWidget/
│   │       ├── ChatWidget.tsx           # Main component
│   │       ├── ChatWidget.module.css    # Scoped styles
│   │       ├── ChatIcon.tsx             # Floating button
│   │       ├── ChatModal.tsx            # Modal container
│   │       ├── MessageList.tsx          # Chat history
│   │       ├── Message.tsx              # Individual message
│   │       ├── MessageInput.tsx         # Input field
│   │       ├── LoadingSpinner.tsx       # Loading indicator
│   │       ├── SelectedTextButton.tsx   # "Ask AI" button
│   │       ├── types.ts                 # TypeScript types
│   │       ├── useChat.ts               # Chat logic hook
│   │       ├── useSelectedText.ts       # Text selection hook
│   │       └── api.ts                   # API client
│   └── theme/
│       └── Root.tsx                     # Docusaurus integration
├── docusaurus.config.js
├── package.json
└── .env.example
```

## 2. Environment Setup

### Step 1: Configure Backend URL

Create a `.env.local` file in the project root:

```bash
cp .env.example .env.local
```

Edit `.env.local` and set the backend URL:

```
REACT_APP_BACKEND_URL=http://localhost:8000
```

For production, use your deployed backend:
```
REACT_APP_BACKEND_URL=https://api.example.com
```

### Step 2: Verify Docusaurus Configuration

Check `docusaurus.config.js` for backend URL configuration (optional):

```javascript
module.exports = {
  // ... other config ...
  customFields: {
    backendUrl: process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000'
  }
};
```

### Step 3: Install Dependencies

```bash
npm install
# or
yarn install
```

## 3. Development Workflow

### Start the Development Server

```bash
npm run start
# or
yarn start
```

This command:
1. Starts the Docusaurus development server (usually on `http://localhost:3000`)
2. Enables hot-reload for code changes
3. Opens the site in your default browser

### Verify Chat Widget is Loaded

1. Navigate to any page in the book
2. Look for the floating chat icon in the bottom-right corner (blue circular button with chat bubble)
3. Click the icon to open the chat modal
4. Type a question and press Enter

### Expected Behavior

- **Chat icon appears** on all pages (bottom-right corner)
- **Icon is responsive**: 60px on desktop, 50px on mobile
- **Click opens modal** smoothly (animation, 300ms)
- **Input field has focus** when modal opens
- **Send button works** (type question → press Enter or click send)
- **Loading spinner appears** while waiting for backend response
- **Response displays** with answer and clickable sources

### Troubleshooting

| Problem | Solution |
|---------|----------|
| Chat icon not visible | Check if ChatWidget is imported in src/theme/Root.tsx |
| Backend connection error | Verify REACT_APP_BACKEND_URL is correct in .env.local |
| Styling issues | Ensure CSS modules are compiled (check browser DevTools) |
| TypeScript errors | Run `npm run typecheck` to see all type errors |

## 4. Testing

### Unit Tests

Run unit tests for Chat Widget components:

```bash
npm run test -- ChatWidget
# or for all tests
npm run test
```

### E2E Tests (Playwright)

Run end-to-end tests:

```bash
npm run test:e2e
```

Tests verify:
- Chat icon visibility and click interaction
- Message sending and response display
- Mobile responsiveness
- Error handling
- Accessibility (keyboard navigation, ARIA labels)

### Manual Testing Checklist

- [ ] Chat icon visible on desktop (60px button)
- [ ] Chat icon visible on mobile (50px button, responsive)
- [ ] Click icon opens modal (smooth animation)
- [ ] Type question and press Enter sends message
- [ ] Loading spinner shows during API call
- [ ] Response displays with answer text
- [ ] Sources appear as clickable links
- [ ] Click source navigates to correct book page
- [ ] Close modal with X button
- [ ] Close modal with Escape key
- [ ] Close modal by clicking outside
- [ ] Conversation history persists after reopening modal
- [ ] Selected text feature works (select text, see "Ask AI" button)
- [ ] Error message shows on backend failure
- [ ] Retry button works after error

## 5. Build for Production

### Create Production Build

```bash
npm run build
```

This command:
1. Builds the Docusaurus site for production
2. Optimizes bundle size (code splitting, minification)
3. Outputs to `build/` directory

### Verify Build Size

```bash
npm run build -- --analyze
```

Check that Chat Widget bundle is ≤ 50KB (minified + gzipped).

### Test Production Build Locally

```bash
npm run serve
```

Open `http://localhost:3000` and verify the chat widget works correctly.

## 6. Configuration

### Backend URL Configuration

**Priority Order**:
1. Environment variable: `REACT_APP_BACKEND_URL`
2. Docusaurus config: `docusaurus.config.js` customFields
3. Fallback: `http://localhost:8000` (development default)

**Example for Different Environments**:

```bash
# Development
REACT_APP_BACKEND_URL=http://localhost:8000

# Staging
REACT_APP_BACKEND_URL=https://staging-api.example.com

# Production
REACT_APP_BACKEND_URL=https://api.example.com
```

### Docusaurus Theme Swizzling

The Chat Widget is integrated into Docusaurus via theme swizzling:

**File**: `src/theme/Root.tsx`

```typescript
import React, { Suspense } from 'react';

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

To disable the Chat Widget temporarily, comment out the ChatWidget component:

```typescript
{/* <ChatWidget /> */}
```

## 7. API Endpoints

The Chat Widget communicates with two backend endpoints:

### POST /api/chat
Ask a question about book content
- **Request**: `{ "question": "What is ROS 2?" }`
- **Response**: `{ "answer": "...", "sources": ["..."] }`

### POST /api/chat-selected
Explain selected text
- **Request**: `{ "question": "...", "selected_text": "..." }`
- **Response**: `{ "answer": "...", "sources": [] }`

See [API Contracts](contracts/api-contracts.md) for full documentation.

## 8. Debugging

### Enable Debug Logging

In `src/components/ChatWidget/ChatWidget.tsx`, uncomment debug logs:

```typescript
useEffect(() => {
  console.log('ChatWidget mounted');
  return () => console.log('ChatWidget unmounted');
}, []);
```

### Check Browser DevTools

1. Open DevTools (F12)
2. Go to **Console** tab:
   - Look for any errors or warnings
   - Chat Widget should log "Component mounted" on load
3. Go to **Network** tab:
   - Monitor API calls to backend
   - Check response status (200 = success, 5xx = error)
   - Verify request payload has correct structure

### Verify Environment Variables

In browser console:
```javascript
console.log(process.env.REACT_APP_BACKEND_URL);
// Should print the backend URL from .env.local
```

## 9. Common Tasks

### Add a New Feature to Chat Widget

1. Create new component file: `src/components/ChatWidget/NewFeature.tsx`
2. Add TypeScript types if needed in `types.ts`
3. Import in parent component (ChatWidget.tsx)
4. Write unit tests: `tests/unit/NewFeature.test.tsx`
5. Update E2E tests if user-facing: `tests/e2e/*.spec.ts`
6. Document in this quickstart if relevant

### Modify Styling

1. Edit `src/components/ChatWidget/ChatWidget.module.css`
2. Use CSS variables for theme colors: `var(--ifm-color-primary)`
3. Follow BEM naming: `.chatWidget__element--modifier`
4. Hot-reload will apply changes (no rebuild needed)

### Test on Mobile

Use browser DevTools device emulation:
1. Open DevTools (F12)
2. Click device toggle (mobile icon)
3. Choose device (iPhone, Android, etc.)
4. Verify responsive layout (< 768px = full-screen modal)

### Performance Profiling

Use React DevTools Profiler:
1. Install React DevTools browser extension
2. Open DevTools → "Profiler" tab
3. Record user interaction (sending message)
4. Check render times and component updates
5. Optimize if any component takes > 50ms

## 10. Deployment

### Deploy to Production

1. Build the site: `npm run build`
2. Set environment variable on hosting platform: `REACT_APP_BACKEND_URL=https://api.example.com`
3. Deploy `build/` directory to your hosting provider (Vercel, Netlify, etc.)
4. Verify chat widget works on production site

### Verify Deployment

On the production site:
- [ ] Chat icon visible
- [ ] Click icon opens modal
- [ ] Send message and receive response
- [ ] No console errors
- [ ] Bundle size under 50KB (check Network tab)

## Next Steps

1. **Explore the codebase**: Read `src/components/ChatWidget/ChatWidget.tsx` to understand component structure
2. **Review specifications**: See [Feature Specification](spec.md) for requirements
3. **Check data model**: See [Data Model](data-model.md) for entity definitions
4. **Run tests**: Execute `npm run test` to ensure everything works
5. **Start developing**: Follow the tasks in `tasks.md` for implementation

## Need Help?

- **TypeScript errors**: Run `npm run typecheck`
- **Build errors**: Delete `node_modules` and `.next` (if exists), then `npm install`
- **Backend not responding**: Verify backend is running on configured URL
- **Styling issues**: Check CSS modules are loaded in browser DevTools

---

**Ready to start?** Run `npm run start` and navigate to the book! 🚀

