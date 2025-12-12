# Physical AI Chatbot Frontend

Simple, lightweight chatbot widget for the Physical AI Assistant.

## Features

- 🎨 Clean, modern UI design
- 💬 Real-time chat with AI assistant
- 📚 Source citations from textbook
- 📱 Fully responsive (mobile & desktop)
- ⚡ Fast, no build process needed
- 🚀 Deploy to GitHub Pages in 1 minute

## Tech Stack

- **Pure HTML/CSS/JavaScript** (No framework dependencies)
- **Backend API:** FastAPI on Render
- **Deployment:** GitHub Pages

## Local Testing

Simply open `index.html` in your browser:

```bash
# Option 1: Direct open
start index.html

# Option 2: Python HTTP server
python -m http.server 8080
# Then visit: http://localhost:8080
```

## Backend Connection

The frontend connects to:
```
https://giaic-hackaton-1-project-1.onrender.com/api/v1
```

Configured in `script.js` line 2:
```javascript
const API_BASE_URL = 'https://giaic-hackaton-1-project-1.onrender.com/api/v1';
```

## Files

- `index.html` - Main chat interface
- `style.css` - Styling and animations
- `script.js` - Chat logic and API integration

## Deployment to GitHub Pages

1. Push code to GitHub
2. Go to repo Settings → Pages
3. Source: Deploy from `master` branch
4. Folder: `/chatbot-frontend`
5. Save

**Live URL:** `https://naveed261.github.io/GIAIC-HACKATON-1-PROJECT-1/`

## Features

### User Interface
- Gradient background design
- Smooth message animations
- Typing indicator while loading
- Auto-scroll to latest message
- Mobile responsive (375px - 1440px)

### Chat Functionality
- Send messages via button or Enter key
- Unique session ID per user
- Source citations with chapter references
- Confidence scores for answers
- Error handling with user-friendly messages

### API Integration
- POST `/api/v1/chat` - Send question
- Session management
- Response parsing
- Error recovery

## Customization

### Change Colors
Edit `style.css` line 12:
```css
background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
```

### Change Backend URL
Edit `script.js` line 2:
```javascript
const API_BASE_URL = 'https://your-backend-url.com/api/v1';
```

### Modify Welcome Message
Edit `index.html` line 19:
```html
<p>👋 Hi! Your custom welcome message here!</p>
```

## Testing Locally

1. Open `index.html` in browser
2. Type: "What is ROS 2?"
3. Check:
   - ✅ Message appears in chat
   - ✅ Typing indicator shows
   - ✅ AI response appears
   - ✅ Sources shown below answer

## Troubleshooting

### CORS Error
If you see CORS errors in console:
- Backend must have CORS enabled
- Check `FRONTEND_URL` in backend .env
- Add your GitHub Pages URL to `ALLOWED_ORIGINS`

### API Not Responding
- Check backend is running: `https://giaic-hackaton-1-project-1.onrender.com/api/v1/health`
- Render free tier sleeps after 15 min (first request takes 30-60s)

### Messages Not Showing
- Open browser DevTools (F12)
- Check Console tab for errors
- Check Network tab for API requests

## Browser Support

- Chrome/Edge: ✅
- Firefox: ✅
- Safari: ✅
- Mobile browsers: ✅

## Production Checklist

- [x] Backend URL configured
- [x] CORS enabled on backend
- [x] Responsive design tested
- [x] Error handling implemented
- [ ] Deployed to GitHub Pages
- [ ] Custom domain (optional)

## Next Steps

1. Deploy to GitHub Pages
2. Test production URL
3. Share chatbot link
4. (Optional) Add custom domain

---

**Live Demo:** Coming soon after GitHub Pages deployment!
