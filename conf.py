# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html
import os
import subprocess
import sys
sys.path.insert(0, os.path.abspath('/home/subhransu/Desktop/RT1_Assignment3-Subhransu/src/RT1_Assignment3-main'))
subprocess.call('doxygen Doxyfile.in', shell=True)
# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'Assignment3_Documentation'
copyright = '2023, Subhransu Priyadarshan'
author = 'Subhransu Priyadarshan'
release = '0.0.1'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
  'sphinx.ext.autodoc',
  'sphinx.ext.doctest',
  'sphinx.ext.intersphinx',
  'sphinx.ext.todo',
  'sphinx.ext.coverage',
  'sphinx.ext.mathjax',
  'sphinx.ext.ifconfig',
  'sphinx.ext.viewcode',
  'sphinx.ext.githubpages',
  "sphinx.ext.napoleon",
  'sphinx.ext.inheritance_diagram',
]

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

autodoc_member_order='bysource'
highlight_language = 'c++'
source_suffix = '.rst'
master_doc = 'index'
html_theme = 'sphinx_rtd_theme'

html_static_path = ['_static']

intersphinx_mapping = {'https://docs.python.org/': None}

todo_include_todos = True
