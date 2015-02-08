import sys
from StringIO import StringIO

class OutputCapture(object):
    def __init__(self):
        self.saved_outputs = dict()

    def _capture_output_with_name(self, output_name):
        self.saved_outputs[output_name] = getattr(sys, output_name)
        setattr(sys, output_name, StringIO())

    def _restore_output_with_name(self, output_name):
        captured_output = getattr(sys, output_name).getvalue()
        setattr(sys, output_name, self.saved_outputs[output_name])
        del self.saved_outputs[output_name]
        return captured_output

    def capture_output(self):
        self._capture_output_with_name("stdout")
        self._capture_output_with_name("stderr")

    def restore_output(self):
        return (self._restore_output_with_name("stdout"), self._restore_output_with_name("stderr"))

    def assert_outputs(self, testcase, function, args=[], kwargs={}, expected_stdout="", expected_stderr=""):
        self.capture_output()
        return_value = function(*args, **kwargs)
        (stdout_string, stderr_string) = self.restore_output()
        testcase.assertEqual(stdout_string, expected_stdout)
        testcase.assertEqual(stderr_string, expected_stderr)
        # This is a little strange, but I don't know where else to return this information.
        return return_value
