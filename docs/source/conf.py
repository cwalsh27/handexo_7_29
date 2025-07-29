# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import os
import sys

# Make sure Python can find your Python module
sys.path.insert(0, os.path.abspath('../../src'))

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information


project = 'NML Hand Exoskeleton'
copyright = '2025, Neuromechatronics Lab'
author = 'Neuromechatronics Lab'
release = '0.0.2'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    "breathe",
    "sphinx.ext.autodoc",
    "sphinx.ext.napoleon",
    "sphinx_rtd_theme",
    "sphinx.ext.autodoc",
    "sphinx.ext.napoleon",
    "sphinx.ext.viewcode",
    "sphinx_autodoc_typehints",
    "sphinx_copybutton",
    "m2r2",
]

templates_path = ['_templates']
exclude_patterns = []
autodoc_typehints = 'description'
autodoc_member_order = 'bysource'
add_module_names = False


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = "sphinx_rtd_theme"
html_static_path = ['_static']
html_logo = "_static/LabLogoRedSquare.png"

# Breathe configuration
breathe_projects = {
    "NMLHandExo": "../build/doxygen/xml"
}
breathe_default_project = "NMLHandExo"

# For Markdown support:
source_suffix = ['.rst', '.md']