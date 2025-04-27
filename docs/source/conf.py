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




# Minimal extensions
extensions = [
    'sphinx.ext.autodoc',      # For Python docstrings
    'sphinx.ext.viewcode',     # Add links to source code
    'sphinx.ext.napoleon'      # Google-style docstrings
]


# RTD Theme settings
html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']


# Theme options
html_theme_options = {
    'navigation_depth': 4,
    'collapse_navigation': False,
    'sticky_navigation': True
}

