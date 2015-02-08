import unittest

from webkitpy.steps.updatechangelogswithreviewer import UpdateChangeLogsWithReviewer
from webkitpy.mock import Mock
from webkitpy.mock_bugzillatool import MockBugzillaTool
from webkitpy.outputcapture import OutputCapture

class UpdateChangeLogsWithReviewerTest(unittest.TestCase):
    def test_guess_reviewer_from_bug(self):
        capture = OutputCapture()
        step = UpdateChangeLogsWithReviewer(MockBugzillaTool(), Mock())
        expected_stderr = "0 reviewed patches on bug 75, cannot infer reviewer.\n"
        capture.assert_outputs(self, step._guess_reviewer_from_bug, [75], expected_stderr=expected_stderr)

    def test_empty_state(self):
        capture = OutputCapture()
        step = UpdateChangeLogsWithReviewer(MockBugzillaTool(), Mock())
        capture.assert_outputs(self, step.run, [{}])
