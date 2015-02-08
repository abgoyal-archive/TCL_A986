import unittest

from webkitpy.steps.closebugforlanddiff import CloseBugForLandDiff
from webkitpy.mock import Mock
from webkitpy.mock_bugzillatool import MockBugzillaTool
from webkitpy.outputcapture import OutputCapture

class CloseBugForLandDiffTest(unittest.TestCase):
    def test_empty_state(self):
        capture = OutputCapture()
        step = CloseBugForLandDiff(MockBugzillaTool(), Mock())
        expected_stderr = "Committed r49824: <http://trac.webkit.org/changeset/49824>\nNo bug id provided.\n"
        capture.assert_outputs(self, step.run, [{"commit_text" : "Mock commit text"}], expected_stderr=expected_stderr)
