# Contributing to MatlabHelper

Thank you for helping us evolve MatlabHelper, the PyQt6 desktop assistant for MATLAB demonstrations. This guide explains how we collaborate effectively, maintain quality, and keep the experience polished for educators.

## Development Workflow

1. **Fork & Clone**

   ```powershell
   git clone https://github.com/<your-username>/MatlabHelper.git
   cd MatlabHelper
   ```

   Create branches from `main` using the convention `feature/<short-name>` or `fix/<short-name>`.

2. **Set Up Python 3.13.7**

   - Install the exact Python version (3.13.7) and ensure it is available on your PATH.
   - Run `scripts/setup_env.ps1` (PowerShell) to create `.venv`, install dependencies, and configure auto-activation.

3. **Coding Standards**

   - Keep files under 500 LOC with clear separation of concerns.
   - Follow PEP 8, add descriptive docstrings, and document non-trivial logic inline.
   - Prefer dependency injection for OS-specific services to keep tests reliable.

4. **Testing**

   - Write unit tests for each new function or class.
   - Add integration tests when touching cross-module flows (e.g., controller/UI interactions).
   - Run the full suite before pushing:
     ```powershell
     pytest
     ```

5. **Documentation**

   - Update the README or Sphinx docs when adding UI flows, settings, or new scripts.
   - Screenshots and short GIFs are welcome for UI changes.

6. **Commit & PR**
   - Use clear, imperative commit messages (e.g., `Add PyQt keyboard monitor`).
   - Submit PRs to `main` with:
     - Summary of changes
     - Testing evidence (command output)
     - Screenshots for UI updates
   - All checks in `.github/workflows/ci-cd.yml` must pass (pytest + MSI build).

## Communication & Review Tips

- Keep PRs focused; open separate PRs for unrelated features.
- Signal blockers early (use Draft PRs or GitHub Discussions).
- Welcome code reviews; annotate tricky parts with comments to guide reviewers.

## Security & Privacy

- Never commit credentials or personal data.
- Use environment variables for secrets; provide `.env.example` updates when necessary.

We’re excited to build a delightful teaching tool together. Thank you for contributing! 🙌
