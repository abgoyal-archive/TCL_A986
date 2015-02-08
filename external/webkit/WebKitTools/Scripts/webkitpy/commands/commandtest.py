import unittest

from webkitpy.mock import Mock
from webkitpy.mock_bugzillatool import MockBugzillaTool
from webkitpy.outputcapture import OutputCapture

class CommandsTest(unittest.TestCase):
    def assert_execute_outputs(self, command, args, expected_stdout="", expected_stderr="", options=Mock(), tool=MockBugzillaTool()):
        command.bind_to_tool(tool)
        OutputCapture().assert_outputs(self, command.execute, [options, args, tool], expected_stdout=expected_stdout, expected_stderr=expected_stderr)
