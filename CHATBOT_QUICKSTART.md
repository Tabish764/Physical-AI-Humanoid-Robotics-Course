# WORKING CHATBOT - QUICK START

## ✅ WHAT'S READY

Your chatbot is **live at http://localhost:3001**

- ✅ Floating chat button (bottom-right of screen)
- ✅ Click to open chat modal
- ✅ Ask questions about the book
- ✅ Get AI answers with source citations
- ✅ Full conversation history
- ✅ Mobile responsive
- ✅ Dark mode support
- ✅ Keyboard accessible

## 🚀 COMMANDS

### Development
```bash
npm start          # Start dev server (http://localhost:3001)
npm run build      # Build production bundle
npm run serve      # Serve built files locally
```

### Testing
```bash
# Manual testing in browser
# 1. Open http://localhost:3001
# 2. Click floating 💬 button (bottom-right)
# 3. Type question about the book
# 4. Hit Enter or click Send button
# 5. Wait for AI response with sources
```

## 📁 KEY FILES

```
src/theme/Root.tsx                     # Mounts widget globally
src/components/ChatWidget/
  ├── ChatWidget.tsx                   # Main container
  ├── ChatIcon.tsx                     # Floating button
  ├── ChatModal.tsx                    # Modal dialog
  ├── MessageList.tsx                  # Chat history
  ├── Message.tsx                      # Message bubble
  └── MessageInput.tsx                 # Form input
src/hooks/useChat.ts                   # Chat state management
src/services/api.ts                    # Backend API client
```

## 🔌 CONFIGURE BACKEND

Edit `.env.local` to point to your backend:

```env
REACT_APP_BACKEND_URL=http://localhost:8000
```

Or set at build time via Docusaurus config.

## 🎨 CUSTOMIZE

### Change Button Position
Edit `src/components/ChatWidget/ChatIcon.module.css`:
```css
bottom: 20px;  /* Distance from bottom */
right: 20px;   /* Distance from right */
```

### Change Colors
Edit `src/components/ChatWidget/base.module.css`:
```css
--chat-widget-primary: #007bff;    /* Button color */
--chat-widget-secondary: #6c757d;  /* Secondary */
--chat-widget-background: white;   /* Modal background */
```

### Change Placeholder Text
Edit `src/components/ChatWidget/MessageInput.tsx`:
```tsx
placeholder="Ask a question about the book..."
```

## 📊 HOW IT WORKS

1. **User clicks button** → `ChatIcon.tsx` opens modal
2. **User types question** → `MessageInput.tsx` captures text
3. **User hits Send** → `useChat.ts` hook calls backend API
4. **API returns response** → `api.ts` client receives answer + sources
5. **Component renders message** → `Message.tsx` displays with citations
6. **Auto-scroll** → `MessageList.tsx` scrolls to newest message
7. **Persist chat** → `useChat.ts` saves to localStorage

## ✨ FEATURES

| Feature | Status | Notes |
|---------|--------|-------|
| Ask question | ✅ | Full text input, character counter |
| View answers | ✅ | With AI-generated responses |
| Show sources | ✅ | Clickable citations/links |
| Conversation history | ✅ | Persisted to localStorage |
| Error handling | ✅ | User-friendly error messages |
| Mobile responsive | ✅ | Touch-friendly design |
| Dark mode | ✅ | Automatic via Docusaurus |
| Keyboard accessible | ✅ | WCAG AA compliant |
| Retry logic | ✅ | Exponential backoff (3 attempts) |
| Timeout handling | ✅ | 30-second timeout with AbortController |

## 🐛 TROUBLESHOOTING

### Button not showing?
- Clear browser cache
- Refresh page (Ctrl+Shift+R)
- Check console for errors (F12)

### API not responding?
- Verify backend is running on port 8000
- Check `.env.local` has correct `REACT_APP_BACKEND_URL`
- Check browser console for error messages

### Chat not persisting?
- Check if localStorage is enabled in browser
- Check browser devtools > Application > Storage > Local Storage

### Build errors?
```bash
rm -r node_modules
rm package-lock.json
npm install
npm run build
```

## 📝 EXAMPLES

### How to add a question
1. Click the 💬 button
2. Type: "What does ROS 2 stand for?"
3. Press Enter or click Send
4. Wait for AI response
5. Click on "Source 1", "Source 2" to view citations

### How to explain selected text
1. Click the 💬 button
2. In the docs, select text from a chapter
3. A button "Ask AI about this" should appear
4. Type your follow-up question
5. AI will answer using the selected text as context

## 🔒 SECURITY

- No API keys stored in frontend
- All backend requests go through your API
- HTTPS recommended for production
- localStorage cleared on browser clear-site-data

## 📈 DEPLOYMENT

### For Production:

```bash
# Build
npm run build

# Deploy build/ folder to your CDN/server
```

### Docusaurus config (docusaurus.config.ts):
```typescript
customFields: {
  backendUrl: process.env.BACKEND_URL || 'https://api.example.com'
}
```

### Docker (optional):
```dockerfile
FROM node:18
WORKDIR /app
COPY . .
RUN npm install && npm run build
CMD ["npm", "run", "serve"]
```

---

**Status**: ✅ READY TO USE

Need to add more features? Check `/specs/002-chat-widget-frontend/tasks.md` for the full task breakdown.

