# 🚀 Render Deployment Guide - Production Backend

Complete step-by-step guide to deploy your RAG Chatbot backend to Render (FREE).

---

## ✅ Why Render?

- ✅ **750 hours/month FREE** (pura month 24/7 chal sakta hai)
- ✅ **GitHub auto-deploy** (code push karo, auto deploy)
- ✅ **Zero config needed** (FastAPI automatically detect)
- ✅ **Free SSL** (HTTPS automatically)
- ✅ **Better uptime** than Railway free tier
- ⚠️ **Sleep after 15 min** inactivity (first request 30-60 sec)

---

## 📋 Prerequisites

✅ GitHub account with repository pushed
✅ Local backend tested and working
✅ OpenAI API key ready
✅ Qdrant Cloud URL and API key
✅ Neon Postgres database URL

---

## 🚀 Deployment Steps (5 Minutes)

### **Step 1: Create Render Account** (1 min)

1. Visit: https://render.com
2. Click **"Get Started"** or **"Sign Up"**
3. Click **"Sign up with GitHub"**
4. Authorize Render to access your repositories
5. Verify email (check inbox)

---

### **Step 2: Create New Web Service** (1 min)

1. On Render Dashboard, click **"New +"** button (top right)
2. Select **"Web Service"**
3. Click **"Build and deploy from a Git repository"**
4. Click **"Next"**

---

### **Step 3: Connect Repository** (1 min)

1. Find your repository: `GIAIC-HACKATON-1-PROJECT-1`
2. Click **"Connect"** button next to it
3. If not visible, click **"Configure account"** and grant access

---

### **Step 4: Configure Service** (2 min)

**Basic Information:**

```
Name: chatbot-backend
Region: Singapore (ya closest available)
Branch: master
Root Directory: chatbot-backend
```

**Build & Deploy Settings:**

```
Runtime: Python 3

Build Command:
pip install -r requirements.txt

Start Command:
uvicorn app.main:app --host 0.0.0.0 --port $PORT
```

**Instance Type:**

```
Instance Type: Free
```

---

### **Step 5: Add Environment Variables** (2 min)

Scroll down to **"Environment Variables"** section.

Click **"Add Environment Variable"** and add these one by one:

#### **OpenAI**
```
Key: OPENAI_API_KEY
Value: [Copy from your .env file]
```

#### **Qdrant**
```
Key: QDRANT_URL
Value: [Copy from your .env file]
```

```
Key: QDRANT_API_KEY
Value: [Copy from your .env file]
```

#### **Database**
```
Key: DATABASE_URL
Value: [Copy from your Neon Postgres connection string]
```

#### **Frontend**
```
Key: FRONTEND_URL
Value: https://naveed261.github.io
```

#### **Logging**
```
Key: LOG_LEVEL
Value: INFO
```

```
Key: LOG_FORMAT
Value: json
```

---

### **Step 6: Deploy!** (Auto)

1. Click **"Create Web Service"** button at bottom
2. Render will automatically:
   - Clone your repository
   - Install dependencies
   - Start your FastAPI server
3. Watch the logs in real-time

**Deployment Logs (Success):**
```
==> Installing dependencies
==> pip install -r requirements.txt
Successfully installed fastapi uvicorn...
==> Starting service
INFO: Starting Physical AI Chatbot API...
INFO: Application startup complete.
INFO: Uvicorn running on 0.0.0.0:10000
==> Your service is live 🎉
```

---

### **Step 7: Get Your Production URL**

After successful deployment:

1. At top of page, you'll see your URL:
   ```
   https://chatbot-backend-xxxx.onrender.com
   ```
2. **Copy this URL** - you'll need it!

---

## ✅ Verify Deployment

### **Test 1: Health Check**

Open browser:
```
https://your-app.onrender.com/api/v1/health
```

**Expected Response:**
```json
{
  "status": "healthy",
  "service": "chatbot-backend",
  "version": "1.0.0"
}
```

### **Test 2: API Documentation**

```
https://your-app.onrender.com/api/docs
```

You should see Swagger UI! ✅

### **Test 3: Chat Endpoint (PowerShell)**

```powershell
$url = "https://your-app.onrender.com/api/v1/chat"

$body = @{
    query = "What is ROS 2?"
    session_id = "test-session-123"
} | ConvertTo-Json

Invoke-RestMethod -Uri $url -Method POST -Body $body -ContentType "application/json"
```

**Real AI response milega!** 🎉

---

## ⚠️ Important: Free Tier Limitations

### **Sleep Mode (15 min inactivity)**

**Problem:**
- 15 minutes inactive = service sleeps
- First request = 30-60 seconds to wake up

**Solution 1: Cron Job (Free)**

Use **cron-job.org** (free service) to ping har 10 minutes:

1. Visit: https://cron-job.org
2. Sign up (free)
3. Create new cron job:
   ```
   URL: https://your-app.onrender.com/api/v1/health
   Interval: Every 10 minutes
   ```
4. Enable job

**Ab aapka service 24/7 awake rahega!** ✅

**Solution 2: Upgrade ($7/month)**
- No sleep
- Always on
- 400 hours/month → Unlimited

---

## 🔧 Troubleshooting

### **Error: Build failed**

**Check:**
1. `requirements.txt` exists in `chatbot-backend/`
2. Root Directory = `chatbot-backend`
3. All packages compatible with Python 3.11+

**Fix:**
- View logs in Render dashboard
- Check requirements.txt format

### **Error: Application failed to start**

**Check:**
1. Start command correct: `uvicorn app.main:app --host 0.0.0.0 --port $PORT`
2. All environment variables added
3. No syntax errors in code

**Fix:**
- Click **"Logs"** tab
- Read error messages
- Fix and redeploy (auto-redeploy on git push)

### **Error: Database connection failed**

**Check:**
1. DATABASE_URL format:
   ```
   postgresql+asyncpg://user:password@host/dbname?sslmode=require
   ```
2. Neon database is active
3. Connection string is complete

### **Error: Health check failing**

**Check:**
1. `/api/v1/health` endpoint exists
2. Server is running on port `$PORT` (Render auto-sets)
3. CORS settings allow requests

---

## 🔄 Auto-Deploy Setup

**Already configured!** Render auto-deploys when you:

1. Make changes locally
2. `git push origin master`
3. Render detects push
4. Auto-rebuilds and redeploys

**View deployments:**
- Click **"Logs"** tab
- See all deployment history

---

## 📊 Render Dashboard Overview

**Overview Tab:**
- Service status (Live/Building/Failed)
- Your URL
- Quick metrics

**Logs Tab:**
- Real-time logs
- Deployment history
- Error messages

**Environment Tab:**
- Environment variables
- Add/edit variables
- Restart required after changes

**Settings Tab:**
- Service name
- Region
- Auto-deploy settings
- Danger zone (delete)

---

## 💡 Pro Tips

### **1. View Real-Time Logs**

```bash
# In Render dashboard
Logs tab → Live logs checkbox
```

### **2. Manual Redeploy**

```
Settings tab → Manual Deploy button
```

### **3. Rollback to Previous Version**

```
Logs tab → Select old deployment → Redeploy
```

### **4. Custom Domain (Free)**

```
Settings → Custom Domains → Add your domain
```

---

## 📝 Post-Deployment Checklist

- [ ] Deployment status = **Live** (green)
- [ ] Health endpoint returns `{"status": "healthy"}`
- [ ] API docs accessible at `/api/docs`
- [ ] Chat endpoint works (real AI responses)
- [ ] All environment variables set
- [ ] Production URL saved
- [ ] Cron job setup (optional - prevents sleep)

---

## 🎯 Your Production URLs

**Backend API:**
```
https://_________________________.onrender.com
```

**API Documentation:**
```
https://_________________________.onrender.com/api/docs
```

**Health Check:**
```
https://_________________________.onrender.com/api/v1/health
```

**Chat Endpoint:**
```
POST https://_________________________.onrender.com/api/v1/chat
```

---

## 📞 Common Issues & Solutions

| Issue | Solution |
|-------|----------|
| Build fails | Check `requirements.txt` and root directory |
| App won't start | Verify start command and environment variables |
| 500 errors | Check logs for Python errors |
| Database errors | Verify DATABASE_URL format |
| Slow first request | Normal - service sleeping (use cron job) |
| Can't access | Check URL is correct (ends with .onrender.com) |

---

## 🚀 Next Steps

1. ✅ **Backend Deployed** - Production URL working
2. ⏭️ **Setup Cron Job** - Prevent sleep (optional)
3. ⏭️ **Test All Endpoints** - Comprehensive testing
4. ⏭️ **Frontend Integration** - Create React widget
5. ⏭️ **Deploy Frontend** - GitHub Pages
6. ⏭️ **End-to-End Testing** - Complete workflow

---

## 📚 Useful Links

**Render Dashboard:**
https://dashboard.render.com

**Render Docs:**
https://render.com/docs

**Cron Job Service:**
https://cron-job.org

**GitHub Repository:**
https://github.com/NAVEED261/GIAIC-HACKATON-1-PROJECT-1

---

## 💰 Render Free Tier

**Included:**
- 750 hours/month (enough for 24/7 with one service)
- 512 MB RAM
- 0.1 CPU
- Free SSL
- Auto-deploy from GitHub
- Custom domains

**Limitations:**
- Sleeps after 15 min inactivity
- Slower than paid tiers
- 100 GB bandwidth/month

---

**🎉 Ready to deploy?** Follow the steps above and your backend will be live in 5 minutes! 🚀
