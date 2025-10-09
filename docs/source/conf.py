import os
import sys
from datetime import datetime

sys.path.insert(0, os.path.abspath("../.."))

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.napoleon",
    "sphinx.ext.viewcode",
]

templates_path = ["_templates"]
exclude_patterns: list[str] = []

html_theme = "sphinx_rtd_theme"
html_static_path = ["_static"]

project = "MatlabHelper"
copyright = "2025, Christian Rickert"
author = "Christian Rickert"
release = "1.0.0"
current_year = datetime.now().year
if str(current_year) not in copyright:
    copyright = f"{current_year}, Christian Rickert"
