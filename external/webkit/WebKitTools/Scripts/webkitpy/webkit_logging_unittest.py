import os
import subprocess
import StringIO
import tempfile
import unittest

from webkitpy.executive import ScriptError
from webkitpy.webkit_logging import *

class LoggingTest(unittest.TestCase):

    def assert_log_equals(self, log_input, expected_output):
        original_stderr = sys.stderr
        test_stderr = StringIO.StringIO()
        sys.stderr = test_stderr

        try:
            log(log_input)
            actual_output = test_stderr.getvalue()
        finally:
            original_stderr = original_stderr

        self.assertEquals(actual_output, expected_output, "log(\"%s\") expected: %s actual: %s" % (log_input, expected_output, actual_output))

    def test_log(self):
        self.assert_log_equals("test", "test\n")

        # Test that log() does not throw an exception when passed an object instead of a string.
        self.assert_log_equals(ScriptError(message="ScriptError"), "ScriptError\n")


if __name__ == '__main__':
    unittest.main()
