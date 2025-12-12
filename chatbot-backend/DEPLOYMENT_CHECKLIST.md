# ✅ Railway Deployment Checklist

Quick reference guide for deploying to Railway.

---

## 📋 Pre-Deployment Checklist

- [x] Local backend tested successfully
- [x] All environment variables ready:
  - [x] OPENAI_API_KEY
  - [x] QDRANT_URL
  - [x] QDRANT_API_KEY
  - [x] DATABASE_URL (Neon Postgres)
  - [x] FRONTEND_URL
- [x] GitHub repository pushed
- [x] `requirements.txt` exists
- [x] `railway.json` configured

---

## 🚀 Deployment Steps

### **1. Railway Setup** (2 min)

- [ ] Go to https://railway.app
- [ ] Login with GitHub
- [ ] Click "New Project"
- [ ] Select "Deploy from GitHub repo"
- [ ] Choose: `GIAIC-HACKATON-1-PROJECT-1`

### **2. Configure Project** (3 min)

- [ ] Click on deployed service
- [ ] Go to **Settings** tab
- [ ] Set **Root Directory**: `chatbot-backend`
- [ ] Verify **Start Command**: `uvicorn app.main:app --host 0.0.0.0 --port $PORT`
- [ ] Click **Save**

### **3. Add Environment Variables** (3 min)

- [ ] Click **Variables** tab
- [ ] Add **OPENAI_API_KEY**
- [ ] Add **QDRANT_URL**
- [ ] Add **QDRANT_API_KEY**
- [ ] Add **DATABASE_URL**
- [ ] Add **FRONTEND_URL**
- [ ] Add **LOG_LEVEL** = `INFO`
- [ ] Add **LOG_FORMAT** = `json`

### **4. Deploy & Monitor** (2 min)

- [ ] Click **Deploy** (if not auto-deployed)
- [ ] Go to **Deployments** tab
- [ ] Watch logs for success messages
- [ ] Wait for "Application startup complete"

### **5. Generate Domain** (1 min)

- [ ] Go to **Settings** tab
- [ ] Scroll to **Domains** section
- [ ] Click **Generate Domain**
- [ ] Copy your URL: `https://______.up.railway.app`

---

## ✅ Post-Deployment Testing

### **Test 1: Health Check**

- [ ] Open: `https://your-app.up.railway.app/api/v1/health`
- [ ] Verify response: `{"status": "healthy"}`

### **Test 2: API Docs**

- [ ] Open: `https://your-app.up.railway.app/api/docs`
- [ ] Verify Swagger UI loads

### **Test 3: Chat Endpoint**

- [ ] Use Swagger UI "Try it out"
- [ ] Test query: "What is ROS 2?"
- [ ] Verify AI response received

---

## 📊 Success Criteria

✅ **Deployment Status**: Green (Active)
✅ **Health Endpoint**: Returns "healthy"
✅ **API Docs**: Accessible and functional
✅ **Chat Endpoint**: Returns AI responses
✅ **Logs**: No errors in deployment logs
✅ **Domain**: Custom domain generated

---

## 🎯 Your URLs (Fill After Deployment)

**Production Backend:**
```
https://_________________________.up.railway.app
```

**API Documentation:**
```
https://_________________________.up.railway.app/api/docs
```

**Health Check:**
```
https://_________________________.up.railway.app/api/v1/health
```

---

## ❌ Common Issues & Fixes

| Issue | Fix |
|-------|-----|
| Build fails | Check root directory = `chatbot-backend` |
| Application failed to respond | Verify all environment variables |
| Database connection error | Check DATABASE_URL format |
| Port binding error | Railway auto-sets $PORT (no action needed) |
| Module not found | Push updated `requirements.txt` |

---

## 📞 Quick Links

**Railway Dashboard:**
https://railway.app/dashboard

**Railway Documentation:**
https://docs.railway.app

**GitHub Repository:**
https://github.com/NAVEED261/GIAIC-HACKATON-1-PROJECT-1

---

## ⏭️ Next Steps After Deployment

1. ✅ Save production URL
2. ⏭️ Test all endpoints thoroughly
3. ⏭️ Update CORS settings if needed
4. ⏭️ Create React frontend widget
5. ⏭️ Deploy frontend to GitHub Pages
6. ⏭️ Full integration testing

---

**Date Deployed:** _______________

**Production URL:** _______________

**Status:** _______________

---

🎉 **Ready to deploy?** Follow the steps above and check them off as you go!
