#!/usr/bin/env python
"""Set up script for mod_pywebsocket.
"""


from distutils.core import setup
import sys


_PACKAGE_NAME = 'mod_pywebsocket'

if sys.version < '2.3':
    print >>sys.stderr, '%s requires Python 2.3 or later.' % _PACKAGE_NAME
    sys.exit(1)

setup(author='Yuzo Fujishima',
      author_email='yuzo@chromium.org',
      description='Web Socket extension for Apache HTTP Server.',
      long_description=(
              'mod_pywebsocket is an Apache HTTP Server extension for '
              'Web Socket (http://tools.ietf.org/html/'
              'draft-hixie-thewebsocketprotocol). '
              'See mod_pywebsocket/__init__.py for more detail.'),
      license='See COPYING',
      name=_PACKAGE_NAME,
      packages=[_PACKAGE_NAME],
      url='http://code.google.com/p/pywebsocket/',
      version='0.4.8',
      )


# vi:sts=4 sw=4 et
