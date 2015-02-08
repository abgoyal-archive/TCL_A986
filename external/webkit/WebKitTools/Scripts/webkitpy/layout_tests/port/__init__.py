#!/usr/bin/env python
"""Port-specific entrypoints for the layout tests test infrastructure."""


import sys


def get(port_name=None, options=None):
    """Returns an object implementing the Port interface. If
    port_name is None, this routine attempts to guess at the most
    appropriate port on this platform."""
    port_to_use = port_name
    if port_to_use is None:
        if sys.platform == 'win32':
            port_to_use = 'chromium-win'
        elif sys.platform == 'linux2':
            port_to_use = 'chromium-linux'
        elif sys.platform == 'darwin':
            port_to_use = 'chromium-mac'

    if port_to_use == 'test':
        import test
        return test.TestPort(port_name, options)
    elif port_to_use.startswith('mac'):
        import mac
        return mac.MacPort(port_name, options)
    elif port_to_use.startswith('chromium-mac'):
        import chromium_mac
        return chromium_mac.ChromiumMacPort(port_name, options)
    elif port_to_use.startswith('chromium-linux'):
        import chromium_linux
        return chromium_linux.ChromiumLinuxPort(port_name, options)
    elif port_to_use.startswith('chromium-win'):
        import chromium_win
        return chromium_win.ChromiumWinPort(port_name, options)

    raise NotImplementedError('unsupported port: %s' % port_to_use)
