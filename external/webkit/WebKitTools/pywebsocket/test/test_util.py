#!/usr/bin/env python
"""Tests for util module."""


import unittest

import config  # This must be imported before mod_pywebsocket.
from mod_pywebsocket import util


class UtilTest(unittest.TestCase):
    def test_get_stack_trace(self):
        self.assertEqual('None\n', util.get_stack_trace())
        try:
            a = 1 / 0  # Intentionally raise exception.
        except Exception:
            trace = util.get_stack_trace()
            self.failUnless(trace.startswith('Traceback'))
            self.failUnless(trace.find('ZeroDivisionError') != -1)

    def test_prepend_message_to_exception(self):
        exc = Exception('World')
        self.assertEqual('World', str(exc))
        util.prepend_message_to_exception('Hello ', exc)
        self.assertEqual('Hello World', str(exc))


if __name__ == '__main__':
    unittest.main()


# vi:sts=4 sw=4 et
