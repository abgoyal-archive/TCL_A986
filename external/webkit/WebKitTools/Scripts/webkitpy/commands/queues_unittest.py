import os

from webkitpy.commands.commandtest import CommandsTest
from webkitpy.commands.queues import *
from webkitpy.commands.queuestest import QueuesTest
from webkitpy.mock_bugzillatool import MockBugzillaTool
from webkitpy.outputcapture import OutputCapture


class TestQueue(AbstractQueue):
    name = "test-queue"


class TestReviewQueue(AbstractReviewQueue):
    name = "test-review-queue"


class AbstractQueueTest(CommandsTest):
    def _assert_log_progress_output(self, patch_ids, progress_output):
        OutputCapture().assert_outputs(self, TestQueue().log_progress, [patch_ids], expected_stderr=progress_output)

    def test_log_progress(self):
        self._assert_log_progress_output([1,2,3], "3 patches in test-queue [1, 2, 3]\n")
        self._assert_log_progress_output(["1","2","3"], "3 patches in test-queue [1, 2, 3]\n")
        self._assert_log_progress_output([1], "1 patch in test-queue [1]\n")

    def _assert_run_webkit_patch(self, run_args):
        queue = TestQueue()
        tool = MockBugzillaTool()
        queue.bind_to_tool(tool)

        queue.run_webkit_patch(run_args)
        expected_run_args = ["echo", "--status-host=example.com"] + map(str, run_args)
        tool.executive.run_and_throw_if_fail.assert_called_with(expected_run_args)

    def test_run_webkit_patch(self):
        self._assert_run_webkit_patch([1])
        self._assert_run_webkit_patch(["one", 2])


class AbstractReviewQueueTest(CommandsTest):
    def test_patch_collection_delegate_methods(self):
        queue = TestReviewQueue()
        tool = MockBugzillaTool()
        queue.bind_to_tool(tool)
        self.assertEquals(queue.collection_name(), "test-review-queue")
        self.assertEquals(queue.fetch_potential_patch_ids(), [103])
        queue.status_server()
        self.assertTrue(queue.is_terminal_status("Pass"))
        self.assertTrue(queue.is_terminal_status("Fail"))
        self.assertTrue(queue.is_terminal_status("Error: Your patch exploded"))
        self.assertFalse(queue.is_terminal_status("Foo"))


class CommitQueueTest(QueuesTest):
    def test_commit_queue(self):
        expected_stderr = {
            "begin_work_queue" : "CAUTION: commit-queue will discard all local changes in \"%s\"\nRunning WebKit commit-queue.\n" % os.getcwd(),
            # FIXME: The commit-queue warns about bad committers twice.  This is due to the fact that we access Attachment.reviewer() twice and it logs each time.
            "next_work_item" : """Warning, attachment 128 on bug 42 has invalid committer (non-committer@example.com)
Warning, attachment 128 on bug 42 has invalid committer (non-committer@example.com)
2 patches in commit-queue [197, 106]
""",
        }
        self.assert_queue_outputs(CommitQueue(), expected_stderr=expected_stderr)


class StyleQueueTest(QueuesTest):
    def test_style_queue(self):
        expected_stderr = {
            "begin_work_queue" : "CAUTION: style-queue will discard all local changes in \"%s\"\nRunning WebKit style-queue.\n" % os.getcwd(),
            "handle_unexpected_error" : "Mock error message\n",
        }
        self.assert_queue_outputs(StyleQueue(), expected_stderr=expected_stderr)
