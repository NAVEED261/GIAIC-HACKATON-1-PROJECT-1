# ============================================
# Backend Installation Script
# ============================================
# This script installs all required dependencies
# ============================================

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "RAG Chatbot Backend - Installation" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""

# Check Python version
Write-Host "[Step 1/4] Checking Python version..." -ForegroundColor Yellow
$pythonVersion = python --version 2>&1
Write-Host "   Found: $pythonVersion" -ForegroundColor Green

if ($LASTEXITCODE -ne 0) {
    Write-Host "   ERROR: Python not found. Please install Python 3.11+" -ForegroundColor Red
    exit 1
}
Write-Host ""

# Upgrade pip
Write-Host "[Step 2/4] Upgrading pip..." -ForegroundColor Yellow
python -m pip install --upgrade pip --quiet
Write-Host "   ✅ pip upgraded successfully" -ForegroundColor Green
Write-Host ""

# Install dependencies
Write-Host "[Step 3/4] Installing dependencies from requirements.txt..." -ForegroundColor Yellow
Write-Host "   This may take 1-2 minutes..." -ForegroundColor Gray
pip install -r requirements.txt

if ($LASTEXITCODE -ne 0) {
    Write-Host "   ❌ Installation failed!" -ForegroundColor Red
    exit 1
}

Write-Host "   ✅ All dependencies installed successfully!" -ForegroundColor Green
Write-Host ""

# Verify critical packages
Write-Host "[Step 4/4] Verifying installation..." -ForegroundColor Yellow

$packages = @(
    "fastapi",
    "uvicorn",
    "sqlalchemy",
    "aiosqlite",
    "openai",
    "qdrant-client",
    "pydantic"
)

$allInstalled = $true
foreach ($package in $packages) {
    $result = pip show $package 2>&1 | Out-Null
    if ($LASTEXITCODE -eq 0) {
        Write-Host "   ✅ $package" -ForegroundColor Green
    } else {
        Write-Host "   ❌ $package (missing)" -ForegroundColor Red
        $allInstalled = $false
    }
}

Write-Host ""

if ($allInstalled) {
    Write-Host "========================================" -ForegroundColor Cyan
    Write-Host "✅ Installation Complete!" -ForegroundColor Green
    Write-Host "========================================" -ForegroundColor Cyan
    Write-Host ""
    Write-Host "Next steps:" -ForegroundColor Yellow
    Write-Host "1. Start server: uvicorn app.main:app --reload" -ForegroundColor White
    Write-Host "2. Open browser: http://127.0.0.1:8000/api/docs" -ForegroundColor White
    Write-Host "3. Run tests: .\test-local.ps1" -ForegroundColor White
    Write-Host ""
} else {
    Write-Host "❌ Some packages failed to install" -ForegroundColor Red
    Write-Host "Try running: pip install -r requirements.txt" -ForegroundColor Yellow
    exit 1
}

Write-Host "Press any key to continue..." -ForegroundColor Gray
$null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")
