# 🚂 Railway Deployment Guide - Production Backend

Complete step-by-step guide to deploy your RAG Chatbot backend to Railway.

---

## 📋 Prerequisites

✅ GitHub account with repository
✅ Local backend tested and working
✅ OpenAI API key
✅ Qdrant Cloud URL and API key
✅ Neon Postgres database URL

---

## 🚀 Step-by-Step Deployment (10 Minutes)

### **Step 1: Create Railway Account**

1. Visit: https://railway.app
2. Click **"Start a New Project"**
3. Click **"Login with GitHub"**
4. Authorize Railway to access your GitHub

---

### **Step 2: Create New Project**

1. Click **"New Project"**
2. Select **"Deploy from GitHub repo"**
3. Choose repository: `GIAIC-HACKATON-1-PROJECT-1`
4. Railway will automatically detect your project

---

### **Step 3: Configure Build Settings**

**Important:** Railway needs to know where backend code is located.

1. Click on your deployed service
2. Go to **"Settings"** tab
3. Find **"Root Directory"** section
4. Set root directory: `chatbot-backend`
5. Click **"Save"**

**Build Configuration:**
- **Root Directory:** `chatbot-backend`
- **Start Command:** `uvicorn app.main:app --host 0.0.0.0 --port $PORT`

---

### **Step 4: Add Environment Variables**

Click **"Variables"** tab, then add these one by one:

#### **OpenAI Configuration**

```
OPENAI_API_KEY
[Copy from your .env file - starts with sk-proj-...]
```

#### **Qdrant Configuration**

```
QDRANT_URL
[Copy from your .env file - Qdrant Cloud URL]
```

```
QDRANT_API_KEY
[Copy from your .env file - Qdrant API key]
```

#### **Database Configuration**

```
DATABASE_URL
[Your Neon Postgres Connection String]
```

**Format:**
```
postgresql+asyncpg://user:password@ep-xxx.region.aws.neon.tech/dbname?sslmode=require
```

#### **Frontend Configuration**

```
FRONTEND_URL
https://naveed261.github.io
```

#### **Logging Configuration**

```
LOG_LEVEL
INFO
```

```
LOG_FORMAT
json
```

---

### **Step 5: Deploy!**

1. After adding all variables, click **"Deploy"**
2. Railway will:
   - Install dependencies from `requirements.txt`
   - Run database migrations (if configured)
   - Start your FastAPI server
3. Wait 2-3 minutes for deployment to complete

**Watch the logs:**
- Click **"Deployments"** tab
- Click on the active deployment
- View real-time logs

**Success Logs:**
```
INFO: Starting Physical AI Chatbot API...
INFO: OpenAI Model: gpt-4o-mini
INFO: Qdrant Collection: textbook_chunks
INFO: Application startup complete.
INFO: Uvicorn running on 0.0.0.0:$PORT
```

---

### **Step 6: Get Your Production URL**

1. Go to **"Settings"** tab
2. Scroll to **"Domains"** section
3. Click **"Generate Domain"**
4. Your URL will be: `https://your-app-name.up.railway.app`

**Copy this URL!** You'll need it for frontend integration.

---

## ✅ Verify Deployment

### **Test 1: Health Check**

Open browser:
```
https://your-app-name.up.railway.app/api/v1/health
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

Open browser:
```
https://your-app-name.up.railway.app/api/docs
```

You should see Swagger UI with all endpoints! ✅

### **Test 3: Chat Endpoint (PowerShell)**

```powershell
$url = "https://your-app-name.up.railway.app/api/v1/chat"

$body = @{
    query = "What is ROS 2?"
    session_id = "test-session-123"
} | ConvertTo-Json

Invoke-RestMethod -Uri $url -Method POST -Body $body -ContentType "application/json"
```

**Expected:** Real AI response with sources! 🎉

---

## 🔧 Troubleshooting

### **Error: Application failed to respond**

**Check:**
1. All environment variables are set correctly
2. Root directory is `chatbot-backend`
3. Start command is correct
4. View deployment logs for errors

**Fix:**
- Go to **"Deployments"** → Click failed deployment → Read logs
- Common issue: Missing environment variable

### **Error: Database connection failed**

**Check:**
1. DATABASE_URL format is correct
2. Neon database is active
3. Connection string has `?sslmode=require`

**Fix:**
```
postgresql+asyncpg://user:password@ep-xxx.region.aws.neon.tech/dbname?sslmode=require
```

### **Error: Module not found**

**Check:**
1. `requirements.txt` exists in `chatbot-backend/`
2. All dependencies listed

**Fix:**
- Commit and push updated `requirements.txt`
- Railway will auto-redeploy

---

## 📊 Railway Dashboard Overview

**Deployments Tab:**
- See all deployments
- View logs
- Rollback if needed

**Settings Tab:**
- Root directory
- Start command
- Custom domains
- Danger zone (delete)

**Variables Tab:**
- Environment variables
- Secrets management

**Metrics Tab:**
- CPU usage
- Memory usage
- Request count

---

## 💰 Railway Pricing

**Free Tier:**
- $5 USD free credit per month
- ~500 hours runtime
- Perfect for testing and small projects

**After Free Tier:**
- Pay-as-you-go: ~$0.000463/GB-hour
- Typical cost: $5-20/month

---

## 🎯 Post-Deployment Checklist

- [ ] Deployment successful (green status)
- [ ] Health endpoint returns "healthy"
- [ ] API docs accessible
- [ ] Chat endpoint works with real queries
- [ ] Environment variables all set
- [ ] Domain generated and working
- [ ] Production URL saved

---

## 📝 Your Production URLs

After deployment, note these URLs:

**Backend API:**
```
https://your-app-name.up.railway.app
```

**API Documentation:**
```
https://your-app-name.up.railway.app/api/docs
```

**Health Check:**
```
https://your-app-name.up.railway.app/api/v1/health
```

**Chat Endpoint:**
```
POST https://your-app-name.up.railway.app/api/v1/chat
```

---

## 🚀 Next Steps After Deployment

1. ✅ **Backend Deployed** - Production URL working
2. ⏭️ **Test All Endpoints** - Verify functionality
3. ⏭️ **Frontend Integration** - Create React widget
4. ⏭️ **Deploy Frontend** - GitHub Pages
5. ⏭️ **End-to-End Testing** - Full workflow

---

## 📞 Need Help?

**Common Issues:**
- Wrong root directory → Set to `chatbot-backend`
- Missing variables → Add all from .env file
- Build failures → Check logs in Deployments tab
- Port issues → Railway auto-sets $PORT variable

**Railway Documentation:**
- https://docs.railway.app

---

**🎉 Deployment Complete!** Your RAG Chatbot backend is now live on Railway! 🚀
