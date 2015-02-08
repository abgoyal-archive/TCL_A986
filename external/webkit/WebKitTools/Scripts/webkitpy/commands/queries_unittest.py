from webkitpy.bugzilla import Bugzilla
from webkitpy.commands.commandtest import CommandsTest
from webkitpy.commands.queries import *
from webkitpy.mock import Mock
from webkitpy.mock_bugzillatool import MockBugzillaTool

class QueryCommandsTest(CommandsTest):
    def test_bugs_to_commit(self):
        expected_stderr = "Warning, attachment 128 on bug 42 has invalid committer (non-committer@example.com)\n"
        self.assert_execute_outputs(BugsToCommit(), None, "42\n77\n", expected_stderr)

    def test_patches_in_commit_queue(self):
        expected_stdout = "http://example.com/197\nhttp://example.com/103\n"
        expected_stderr = "Warning, attachment 128 on bug 42 has invalid committer (non-committer@example.com)\nPatches in commit queue:\n"
        self.assert_execute_outputs(PatchesInCommitQueue(), None, expected_stdout, expected_stderr)

    def test_patches_to_commit_queue(self):
        expected_stdout = "http://example.com/104&action=edit\n"
        expected_stderr = "197 already has cq=+\n128 already has cq=+\n105 committer = \"Eric Seidel\" <eric@webkit.org>\n"
        options = Mock()
        options.bugs = False
        self.assert_execute_outputs(PatchesToCommitQueue(), None, expected_stdout, expected_stderr, options=options)

        expected_stdout = "http://example.com/77\n"
        options.bugs = True
        self.assert_execute_outputs(PatchesToCommitQueue(), None, expected_stdout, expected_stderr, options=options)

    def test_patches_to_review(self):
        expected_stdout = "103\n"
        expected_stderr = "Patches pending review:\n"
        self.assert_execute_outputs(PatchesToReview(), None, expected_stdout, expected_stderr)

    def test_tree_status(self):
        expected_stdout = "ok   : Builder1\nok   : Builder2\n"
        self.assert_execute_outputs(TreeStatus(), None, expected_stdout)
