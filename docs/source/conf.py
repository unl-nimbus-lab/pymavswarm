import os
import sys

import furo  # noqa: F401

sys.path.insert(0, os.path.abspath("../../pymavswarm/"))

project = "pymavswarm"
author = "Evan Palmer"
copyright = f"2022, {author}"

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.autosummary",
]

templates_path = ["_templates"]
exclude_patterns = ["_build"]

epub_description = "Python library for drone swarm control and algorithm development"

html_theme = "furo"
html_static_path = []  # type: ignore
