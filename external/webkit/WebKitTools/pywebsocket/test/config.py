"""Configuration for testing.

Test files should import this module before mod_pywebsocket.
"""


import os
import sys


# Add the parent directory to sys.path to enable importing mod_pywebsocket.
sys.path += [os.path.join(os.path.split(__file__)[0], '..')]


# vi:sts=4 sw=4 et
