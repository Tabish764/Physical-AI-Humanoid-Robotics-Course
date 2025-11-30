<!-- Sync Impact Report (v1.0.0) -->
<!-- Version Change: N/A → 1.0.0 (initial constitution) -->
<!-- Added Sections: Core Principles (6), Docusaurus Integration Standards, AI/ML Integration Standards, Frontend-Backend Communication -->
<!-- Templates Updated: ✅ plan-template.md (Constitution Check section), ✅ spec-template.md (scope alignment), ✅ tasks-template.md (task categorization) -->
<!-- Follow-up TODOs: None -->

# AI-Native Book Platform Constitution

## Core Principles

### I. Docusaurus-First Content Delivery
All book content MUST be delivered through Docusaurus static site generation. The book structure MUST be organized in the `docs/` directory with proper sidebar ordering via `sidebar_position` frontmatter. Content MUST be markdown-based with frontmatter metadata. The platform MUST enhance but never replace Docusaurus core functionality.

### II. Non-Intrusive AI Enhancement Architecture
All AI features (chat widget, text selection, RAG chatbot) MUST be optional enhancements that do not break existing Docusaurus functionality. AI components MUST use theme swizzling via `src/theme/Root.tsx` for integration. No core Docusaurus files (layout.tsx, navbar, sidebar) MAY be modified. All AI features MUST gracefully degrade when backend services are unavailable.

### III. RESTful API Backend with FastAPI
The backend MUST use FastAPI for all API endpoints with proper Pydantic validation. All endpoints MUST follow RESTful conventions with clear request/response models. Error handling MUST use descriptive HTTP status codes. The backend MUST support CORS for Docusaurus frontend integration. Rate limiting awareness MUST be implemented for external AI services (Gemini API).

### IV. Vector-Based RAG for Intelligent Content Retrieval
Book content MUST be embedded using vector embeddings (Gemini text-embedding-004) and stored in Qdrant vector database. RAG queries MUST retrieve top-k similar chunks (default k=3) using cosine similarity. The embedding process MUST be a one-time setup script (`embed_book.py`) that processes all markdown files from `docs/`. Context-aware responses MUST be generated using retrieved chunks as context.

### V. React 18+ with TypeScript for Frontend Components
All frontend components MUST use React 18+ functional components with hooks. TypeScript MUST be used for all component code with strict type checking. Components MUST be modular, reusable, and independently testable. State management MUST use React hooks (useState, useEffect, useContext) without external state libraries for MVP. CSS MUST use CSS modules or scoped styles to prevent leakage.

### VI. Text Selection Context Preservation
When users select text and ask questions, the selected text MUST be sent as context to the backend without requiring vector search. The `/api/chat-selected` endpoint MUST accept `selected_text` and `question` parameters. The AI response MUST be based solely on the provided selected text context. Selected text MUST be preserved in the chat interface for user reference during the conversation.

## Docusaurus Integration Standards

### Theme Swizzling Requirements
- AI components MUST be integrated via `src/theme/Root.tsx` wrapper
- Components MUST persist across page navigation
- No modifications to core Docusaurus files allowed
- CSS variables MUST be used for theme alignment (`--ifm-color-primary`, etc.)

### Content Organization
- All book content in `docs/` directory with proper module structure
- Sidebar ordering controlled via `sidebar_position` frontmatter
- Frontmatter MUST include: `id`, `title`, `sidebar_position` (where applicable)
- Module categories defined via `_category_.json` files

### Performance Standards
- Static site generation MUST complete in reasonable time (<5 minutes for typical book)
- Page load times MUST remain under 2 seconds
- AI widget bundle size MUST NOT exceed 50KB (minified + gzipped)
- No impact on Docusaurus build process or output

## AI/ML Integration Standards

### Vector Database (Qdrant)
- Collection name: `physical_ai_book`
- Vector dimension: 768 (Gemini text-embedding-004)
- Distance metric: Cosine similarity
- Payload schema: `{file_path, module, content, chunk_index}`

### Embedding Process
- Chunk size: 800-1200 characters with 100-character overlap
- Frontmatter MUST be stripped before embedding
- All `.md` files in `docs/` MUST be processed
- Embedding script MUST handle rate limits with exponential backoff

### LLM Integration (Gemini via OpenAI SDK)
- Model: `gemini-2.5-flash` for chat completions
- Embedding model: `text-embedding-004`
- Base URL: `https://generativelanguage.googleapis.com/v1beta/openai/`
- Rate limit awareness: 10 RPM (free tier)
- Retry logic: 3 attempts with exponential backoff
- Max tokens: 2000 for chat responses

### Response Quality Standards
- Responses MUST be based on retrieved context (RAG) or provided selected text
- Empty or None responses MUST trigger retry logic
- Error messages MUST be user-friendly and actionable
- Sources MUST be provided for RAG responses (file paths)

## Frontend-Backend Communication

### API Endpoints
- `POST /api/chat`: RAG-based question answering with vector search
- `POST /api/chat-selected`: Context-based question answering with selected text
- `GET /api/health`: Health check for dependencies (Qdrant, Postgres)

### Request/Response Models
- Request validation via Pydantic models with field constraints
- Response models MUST include `answer` (string) and `sources` (string array)
- Error responses MUST use proper HTTP status codes (400, 500, 503)

### Error Handling
- Network errors: User-friendly messages, retry options
- Timeout: 30-second maximum per request
- Backend unavailable: Graceful degradation, no widget crashes
- Rate limiting: Clear messaging about service availability

### Data Validation
- Question length: 1-2000 characters
- Selected text length: 1-5000 characters
- Input sanitization: Trim whitespace, validate non-empty
- Response validation: Check for None/empty before display

## Development Workflow and Quality Gates

### Code Quality Standards
- **Python**: 3.10+ with type hints on all functions
- **TypeScript**: Strict mode, no `any` types
- **Linting**: ESLint (frontend), flake8/black (backend)
- **Testing**: Unit tests for components, integration tests for API
- **Documentation**: Inline comments, README updates for new features

### Testing Requirements
- Unit tests for React components (React Testing Library)
- Integration tests for API endpoints
- E2E tests for user flows (chat, text selection)
- Accessibility tests (WCAG AA compliance)
- Mobile responsiveness tests (< 768px breakpoint)

### Performance Benchmarks
- API response time: < 5 seconds for chat endpoints
- Widget initial load: < 100ms to interactive
- Modal animations: 300ms smooth transitions
- Bundle size: < 50KB for chat widget
- No memory leaks on long sessions

### Deployment Checklist
- ✅ Environment variables configured (no hardcoded secrets)
- ✅ CORS headers correct on backend
- ✅ TypeScript types strict, no errors
- ✅ CSS scoped, no conflicts with Docusaurus
- ✅ Accessibility audit passed (WCAG AA)
- ✅ Mobile testing on real devices
- ✅ Error messages user-friendly
- ✅ Loading states visible
- ✅ Keyboard navigation works
- ✅ No console errors or warnings

## Constraints and Non-Goals

### Hard Constraints
- MUST work with Docusaurus 2.x and 3.x
- MUST NOT modify core Docusaurus files
- MUST NOT exceed 50KB bundle size for widgets
- MUST be accessible (WCAG AA minimum)
- MUST handle errors gracefully (no crashes)
- MUST support mobile devices (< 768px)
- MUST use free tier services (Qdrant, Neon, Gemini)

### Non-Goals (MVP)
- User authentication UI
- Voice input/output
- File upload capability
- Multi-language UI (English only)
- Conversation history sync across devices
- Real-time typing indicators
- Message editing/deletion
- Custom dark mode (use Docusaurus theme)

## Governance

### Amendment Procedure
1. Changes to principles require architectural decision record (ADR)
2. Version bumps follow semantic versioning:
   - **MAJOR**: Principle removals or incompatible changes (e.g., remove Docusaurus requirement)
   - **MINOR**: New principle or major expansion (e.g., add text selection feature)
   - **PATCH**: Clarifications, wording, non-semantic refinements
3. All PRs MUST verify compliance with constitution
4. Code review MUST check against accessibility, performance, and integration standards

### Compliance Review
- **Monthly**: Review open issues/PRs for constitution violations
- **On Release**: Full checklist audit (development workflow section)
- **On Architecture Change**: Re-validate integration constraints
- **On New Feature**: Verify non-intrusive integration and performance impact

### Constitution Supersedes
This constitution supersedes all other development practices. All team members and AI agents MUST follow these principles. When conflicts arise between constitution and other guidance, constitution takes precedence. Use this document as the authoritative source for architectural decisions.

---

**Version**: 1.0.0 | **Ratified**: 2025-11-30 | **Last Amended**: 2025-11-30
