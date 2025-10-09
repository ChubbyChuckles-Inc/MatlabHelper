param(
    [string]$PythonPath = "python",
    [string]$ProjectRoot = (Get-Location)
)

Write-Host "Creating virtual environment with Python 3.13.7..." -ForegroundColor Cyan
& $PythonPath -m venv (Join-Path $ProjectRoot '.venv')

$venvActivate = Join-Path $ProjectRoot '.venv\Scripts\Activate.ps1'
if (-not (Test-Path $venvActivate)) {
    throw "Virtual environment activation script not found: $venvActivate"
}

Write-Host "Installing project dependencies..." -ForegroundColor Cyan
& $PythonPath -m pip install --upgrade pip
& (Join-Path $ProjectRoot '.venv\Scripts\pip.exe') install -r (Join-Path $ProjectRoot 'requirements.txt')

$profilePath = "$HOME\Documents\PowerShell\Microsoft.PowerShell_profile.ps1"
if (-not (Test-Path $profilePath)) {
    New-Item -ItemType File -Path $profilePath -Force | Out-Null
}

$activationBlock = @"
if (Test-Path '$venvActivate') {
    Push-Location '$ProjectRoot'
    . '$venvActivate'
    Pop-Location
}
"@

if (-not (Get-Content $profilePath | Select-String -SimpleMatch $venvActivate)) {
    Add-Content -Path $profilePath -Value $activationBlock
    Write-Host "Added auto-activation snippet to PowerShell profile." -ForegroundColor Green
}
else {
    Write-Host "Auto-activation snippet already present in PowerShell profile." -ForegroundColor Yellow
}

Write-Host "Environment ready. Open a new PowerShell terminal to use the virtual environment automatically." -ForegroundColor Green
