# conf.py
import os
import sys
from datetime import datetime

# Basic project info
project = 'Cooking Robot'
copyright = f'{datetime.now().year}, Your Name'
author = 'Your Name'
release = '1.0'


# Add all necessary paths - including msg and srv directories
base_path = os.path.abspath('../../src')
sys.path.insert(0, base_path)
sys.path.insert(0, os.path.join(base_path, 'controller'))


sys.path.insert(0, os.path.join(base_path, 'perception'))

sys.path.insert(0, os.path.join(base_path, 'cooking_manager'))

# Add this to conf.py if using ROS workspace
sys.path.insert(0, os.path.abspath('../../devel/lib/python3/dist-packages'))




show_authors=True


# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'COGAR aassignment'
copyright = '2025, '
author = 'M M A'
release = '1.0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = ['sphinx.ext.autodoc',
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
'breathe']

templates_path = ['_templates']
exclude_patterns = []



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output


highlight_language = 'c++'
source_suffix = '.rst'
master_doc = 'index'
html_theme = 'sphinx_rtd_theme'
