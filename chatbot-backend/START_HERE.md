# 🚀 START HERE - Quick Setup Guide

Bilkul simple 3 steps mein backend start karein!

---

## ⚡ Super Quick Start (2 Minutes)

### Step 1: Install Dependencies (Ek Baar)

PowerShell mein run karein:

```powershell
.\INSTALL.ps1
```

**Ya manually:**

```powershell
pip install -r requirements.txt
```

**Output (successful installation):**
```
✅ fastapi
✅ uvicorn
✅ sqlalchemy
✅ aiosqlite
✅ openai
✅ qdrant-client
✅ pydantic
```

---

### Step 2: Start Server

```powershell
uvicorn app.main:app --reload
```

**Expected Output (SUCCESS):**
```
INFO:     Uvicorn running on http://127.0.0.1:8000 (Press CTRL+C to quit)
INFO:     Started reloader process using WatchFiles
INFO:     Started server process
INFO:     Waiting for application startup.
INFO:     Starting Physical AI Chatbot API...
INFO:     OpenAI Model: gpt-4o-mini
INFO:     Qdrant Collection: textbook_chunks
INFO:     Rate Limit: 100 req/3600s
INFO:     Application startup complete.
```

---

### Step 3: Test in Browser

Yeh URL kholein:

```
http://127.0.0.1:8000/api/docs
```

**Aapko dikhega:**
- ✅ Interactive API documentation (Swagger UI)
- ✅ Sabhi endpoints test kar sakte hain
- ✅ Try it out button se live testing

---

## 🧪 Quick Test Commands

### Health Check (Terminal mein)

```powershell
Invoke-RestMethod -Uri "http://127.0.0.1:8000/api/v1/health"
```

**Response:**
```json
{
  "status": "healthy",
  "service": "chatbot-backend",
  "version": "1.0.0"
}
```

### Automated Tests

```powershell
.\test-local.ps1
```

---

## ❌ Agar Error Aaye

### Error: "No module named 'aiosqlite'"

**Fix:**
```powershell
pip install aiosqlite
```

### Error: "Port 8000 already in use"

**Fix:**
```powershell
# Different port use karein
uvicorn app.main:app --reload --port 8001
```

### Error: "ModuleNotFoundError"

**Fix:**
```powershell
# Sab packages reinstall karein
pip uninstall -y -r requirements.txt
pip install -r requirements.txt
```

### Error: "Permission denied"

**Fix:**
```powershell
# PowerShell ko admin mode mein run karein
# Right-click PowerShell → Run as Administrator
```

---

## 📋 Checklist

- [ ] Python 3.11+ installed hai
- [ ] PowerShell admin mode mein hai
- [ ] `chatbot-backend/` directory mein hain
- [ ] `.\INSTALL.ps1` run kiya
- [ ] Sabhi packages install hue (✅ dikhne chahiye)
- [ ] `uvicorn app.main:app --reload` run kiya
- [ ] Server start hua (INFO messages dikhe)
- [ ] http://127.0.0.1:8000/api/docs khula
- [ ] Health check passed

---

## 🎯 Exact Commands (Copy-Paste)

```powershell
# 1. Navigate to backend
cd "D:\PIAIC HACKATON PRACTICE\GIAIC-HACKATON-1_PROJECT-1\chatbot-backend"

# 2. Install (one time only)
.\INSTALL.ps1

# 3. Start server
uvicorn app.main:app --reload

# 4. In new terminal - test
Invoke-RestMethod -Uri "http://127.0.0.1:8000/api/v1/health"
```

---

## 📞 Still Having Issues?

1. **Screenshot error message** aur share karein
2. **Python version** check karein: `python --version`
3. **Current directory** verify karein: `pwd`
4. **Installed packages** check karein: `pip list | findstr fastapi`

---

## ✅ Success!

Agar yeh output dikha to **SUCCESS** ✅:

```
INFO:     Application startup complete.
```

Aur browser mein http://127.0.0.1:8000/api/docs khul gaya!

---

**Ab try karein aur result bataiye!** 🚀
