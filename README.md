# MatlabHelper

MatlabHelper is a Windows-first desktop application that speeds up MATLAB demonstrations. Select a MATLAB `.m` script, focus the MATLAB editor, and any random keyboard tap will stream the file's contents directly into the active editor window. The app provides a modern PyQt6 experience, graceful error handling, and automated packaging as an MSI installer.

## Feature Highlights

- 🌌 **Elegant PyQt6 interface** with dark-friendly contrast, responsive layouts, and live status indicators.
- 🪟 **MATLAB editor detection** that keeps track of the active MATLAB window.
- ⌨️ **Random keystroke trigger**: once a key press is detected, the selected MATLAB file is injected into the editor.
- 🛡️ **Robust validation & messaging** for missing files, invalid extensions, or unavailable MATLAB windows.
- 🧪 **Comprehensive testing suite** (unit + integration) powered by `pytest` and `pytest-qt`.
- 📦 **CI/CD pipeline** that exercises tests and emits a Windows `.msi` via `cx_Freeze` on every commit.

## Requirements

- Windows 10/11 with MATLAB installed (editor window required for injection).
- Python **3.13.7** (the project and automation scripts rely on this exact version).
- PowerShell execution policy that allows running local scripts (`RemoteSigned` or less restrictive).

## Quick Start

1. **Install Python 3.13.7**
   Download the official installer from [python.org](https://www.python.org/downloads/windows/) and ensure "Add to PATH" is enabled. Confirm the version:

   ```powershell
   python --version
   ```

2. **Clone the repository**

   ```powershell
   git clone https://github.com/ChubbyChuckles-Inc/MatlabHelper.git
   cd MatlabHelper
   ```

3. **Create and prime the virtual environment**

   ```powershell
   python -m venv .venv
   .\.venv\Scripts\Activate.ps1
   pip install --upgrade pip
   pip install -r requirements.txt
   ```

4. **Auto-activate the virtual environment (PowerShell)**
   Add the snippet below to your PowerShell profile. Replace `F:\Coding\MatlabHelper` with your local clone path.

   ```powershell
   $profilePath = "$HOME\Documents\PowerShell\Microsoft.PowerShell_profile.ps1"
   if (-not (Test-Path $profilePath)) {
      New-Item -ItemType File -Path $profilePath -Force | Out-Null
   }

   $projectRoot = 'F:\Coding\MatlabHelper' # TODO: update to your clone path
   $activateScript = Join-Path $projectRoot '.venv\Scripts\Activate.ps1'
   $profileBlock = @"
   if (Test-Path '$activateScript') {
   Push-Location '$projectRoot'
   . '$activateScript'
   Pop-Location
   }
   "@

   if (-not (Get-Content $profilePath -ErrorAction SilentlyContinue | Select-String -SimpleMatch $activateScript)) {
      Add-Content -Path $profilePath -Value $profileBlock
   }
   ```

   Open a new PowerShell window in the repository to verify that `.venv` activates automatically.

5. **Run the application**
   ```powershell
   python -m src.main
   ```

## Usage

1. Launch MatlabHelper and click **Select MATLAB Script** to choose a `.m` file.
2. Ensure a MATLAB editor window is open and active. Click **Refresh** if needed to detect it.
3. Toggle **Keystroke Listener** on. The app waits silently.
4. Tap any key on the keyboard. MatlabHelper pastes the selected script into the active editor and stops listening.
5. Toggle the listener back on whenever you need to re-run the demo.

## Testing

The suite covers validation logic, window detection (via mocks), keyboard monitoring, and integration flows.

```powershell
pytest
```

For UI-centric tests that rely on Qt, `pytest-qt` handles the Qt event loop automatically.

## Packaging

The CI pipeline uses `cx_Freeze` to build a Windows `.msi`. You can run the same build locally:

```powershell
python setup_cx_freeze.py build
python setup_cx_freeze.py bdist_msi
```

Artifacts publish to `build/` locally and to the GitHub Actions run as an artifact.

## Project Layout

- `src/` — Application code (Qt UI, services, controllers).
- `tests/` — Unit and integration tests executed via `pytest`.
- `docs/` — Sphinx documentation (optional) tailored for MatlabHelper.
- `scripts/` — Helper scripts (environment bootstrap, CI utilities).
- `.github/workflows/ci-cd.yml` — Continuous integration pipeline definition.

## Troubleshooting

- **No MATLAB window detected**: ensure MATLAB is running and the editor window has focus. The status bar should display the detected title.
- **Permission errors**: run PowerShell as administrator when adjusting execution policies or interacting with windows across integrity levels.
- **Keyboard hook not firing**: some antivirus tools block global hooks; whitelist Python or run as admin.
- **Installer build failures**: confirm you are on Windows with Visual C++ Build Tools installed (required by `cx_Freeze`).

## License

MIT License © ChubbyChuckles-Inc. See `LICENSE` for details.
