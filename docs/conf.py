import os
import sys
import inno_control # Install before usage !!! pip install pip install -e ~/total_control/inno_control/
# test
# -- Path Setup --------------------------------------------------------------

# Corrected path to point to your PACKAGE directory (not just parent)
sys.path.insert(0, os.path.abspath('../inno_control'))

# -- Project Information -----------------------------------------------------

project = 'total-control'
copyright = '2025, Dungeon team'
author = 'Dungeon team'
release = 'MVP-0'

# -- General Configuration ---------------------------------------------------

extensions = [
    'sphinx.ext.autodoc',    # Core docstring extraction
    'sphinx.ext.napoleon',   # Google/NumPy docstring support
    'sphinx.ext.viewcode',   # Source code links
    'sphinx.ext.autosummary',# Generate summary tables
    'myst_parser'            # Markdown support
]

# Napoleon settings for better docstring parsing
napoleon_google_docstring = True
napoleon_numpy_docstring = False  # Use Google-style docstrings
napoleon_include_init_with_doc = True
napoleon_include_private_with_doc = False
napoleon_include_special_with_doc = True

# Autodoc settings
autodoc_default_options = {
    'members': True,
    'member-order': 'bysource',
    'special-members': '__init__',
    'undoc-members': True,
    'show-inheritance': True
}
autodoc_typehints = "description"
autodoc_mock_imports = []  # Add problematic dependencies here

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

root_doc = 'index' # Should match your main doc file (index.md/index.rst)

# -- HTML Output -------------------------------------------------------------

html_theme = 'sphinx_rtd_theme'

# Theme options for ReadTheDocs theme
html_theme_options = {
    'navigation_depth': 4,
    'collapse_navigation': False,
    'titles_only': False
}

# -- Final Setup -------------------------------------------------------------

autosummary_generate = True