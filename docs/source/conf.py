import furo  # noqa

project = "pymavswarm"
author = "Evan Palmer"
copyright = f"2022, {author}"

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.autosummary",
]

templates_path = ["_templates"]
exclude_patterns = []  # type: ignore

epub_description = "Python library for drone swarm control and algorithm development"

html_theme = "furo"
html_static_path = ["_static"]
