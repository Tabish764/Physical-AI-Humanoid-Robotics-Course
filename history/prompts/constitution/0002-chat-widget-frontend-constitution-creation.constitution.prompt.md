---
ID: 0002
TITLE: Chat Widget Frontend Constitution Creation
STAGE: constitution
DATE_ISO: 2025-11-29
SURFACE: agent
MODEL: claude-haiku-4.5
FEATURE: none
BRANCH: 002-chat-widget-frontend
USER: user
COMMAND: /sp.constitution
LABELS: ["frontend","react","docusaurus","chat-widget","constitution","chatbot-ui"]
LINKS:
  SPEC: null
  TICKET: null
  ADR: null
  PR: null
FILES_YAML:
  - specs/002-chat-widget-frontend/constitution.md
TESTS_YAML: []
---

## User Prompt

```
/sp.constitution

Project: Chat Widget Component for Docusaurus - Frontend Integration with RAG Backend

Meta Rules:
- The Speckit Constitution, tasks, and spec files live ONLY in the ROOT directory of the repository.
- The chat widget code will be added to Docusaurus src/ directory.
- All AI agents must respect the existing Docusaurus structure and not break it.

Core principles:
- Minimal, non-intrusive floating chat widget (ChatGPT style)
- Seamless integration with existing Docusaurus site
- Clean, modern UI matching Docusaurus theme
- Direct communication with FastAPI backend
- Responsive design for mobile and desktop
- No breaking changes to existing book functionality

Key standards:
- React 18+ functional components with hooks
- TypeScript for type safety (or vanilla JavaScript if simpler)
- CSS modules or inline styles (avoid conflicts with Docusaurus theme)
- Accessible UI with ARIA labels and keyboard navigation
- Error handling with user-friendly messages
- Loading states and visual feedback
- Mobile-first responsive design

Project Structure:
src/
├── components/
│   └── ChatWidget/
│       ├── ChatWidget.tsx (or .jsx)      # Main chat component
│       ├── ChatWidget.module.css         # Scoped styles
│       ├── ChatIcon.tsx                  # Floating button
│       ├── ChatModal.tsx                 # Chat window
│       └── types.ts                      # TypeScript types
├── theme/
│   └── Root.tsx                          # Docusaurus theme wrapper (to inject chat widget globally)

[... full specifications provided ...]
```

## Assistant Response

Created comprehensive Chat Widget Frontend Constitution v1.0.0 with:

**Core Principles (6):**
1. Non-Intrusive Floating Widget Design
2. Seamless Docusaurus Integration (2.x/3.x)
3. Clean, Modern UI Matching Theme
4. Direct Backend Communication with Error Handling
5. Responsive Design: Mobile-First Approach
6. React 18+ with TypeScript and Functional Components

**Major Sections:**
- UI/UX Standards (icon specs, modal specs, message display)
- Backend Integration Standards (API protocol, error handling, data validation)
- Accessibility Standards (ARIA, keyboard nav, WCAG AA compliance)
- Mobile Responsiveness Standards (breakpoints, touch interaction, performance)
- State Management and Component Architecture
- Docusaurus Integration (theme swizzling, environment config, CSS isolation)
- Development Workflow and Quality Gates
- Constraints and Non-Goals

**File Created:**
- `specs/002-chat-widget-frontend/constitution.md` (v1.0.0)

**Version Rationale:**
- New feature, starting at v1.0.0
- MINOR version bump from RAG backend (independent feature)
- Ratified and Amended dates: 2025-11-29

---

## Outcome

Constitution file created successfully at:
`specs/002-chat-widget-frontend/constitution.md`

Ready for next phase: `/sp.spec` to create detailed feature specification.

```
