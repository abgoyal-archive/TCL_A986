import unittest

from webkitpy.bugzilla import Attachment
from webkitpy.mock import Mock
from webkitpy.mock_bugzillatool import MockBugzillaTool
from webkitpy.outputcapture import OutputCapture


class MockQueueEngine(object):
    def __init__(self, name, queue):
        pass

    def run(self):
        pass


class QueuesTest(unittest.TestCase):
    mock_work_item = Attachment({
        "id" : 1234,
        "bug_id" : 345,
        "attacher_email": "adam@example.com",
    }, None)

    def assert_queue_outputs(self, queue, args=None, work_item=None, expected_stdout=None, expected_stderr=None, options=Mock(), tool=MockBugzillaTool()):
        if not expected_stdout:
            expected_stdout = {}
        if not expected_stderr:
            expected_stderr = {}
        if not args:
            args = []
        if not work_item:
            work_item = self.mock_work_item
        tool.user.prompt = lambda message: "yes"

        queue.execute(options, args, tool, engine=MockQueueEngine)

        OutputCapture().assert_outputs(self,
                queue.queue_log_path,
                expected_stdout=expected_stdout.get("queue_log_path", ""),
                expected_stderr=expected_stderr.get("queue_log_path", ""))
        OutputCapture().assert_outputs(self,
                queue.work_item_log_path,
                args=[work_item],
                expected_stdout=expected_stdout.get("work_item_log_path", ""),
                expected_stderr=expected_stderr.get("work_item_log_path", ""))
        OutputCapture().assert_outputs(self,
                queue.begin_work_queue,
                expected_stdout=expected_stdout.get("begin_work_queue", ""),
                expected_stderr=expected_stderr.get("begin_work_queue", ""))
        OutputCapture().assert_outputs(self,
                queue.should_continue_work_queue,
                expected_stdout=expected_stdout.get("should_continue_work_queue", ""), expected_stderr=expected_stderr.get("should_continue_work_queue", ""))
        OutputCapture().assert_outputs(self,
                queue.next_work_item,
                expected_stdout=expected_stdout.get("next_work_item", ""),
                expected_stderr=expected_stderr.get("next_work_item", ""))
        OutputCapture().assert_outputs(self,
                queue.should_proceed_with_work_item,
                args=[work_item],
                expected_stdout=expected_stdout.get("should_proceed_with_work_item", ""),
                expected_stderr=expected_stderr.get("should_proceed_with_work_item", ""))
        OutputCapture().assert_outputs(self,
                queue.process_work_item,
                args=[work_item],
                expected_stdout=expected_stdout.get("process_work_item", ""),
                expected_stderr=expected_stderr.get("process_work_item", ""))
        OutputCapture().assert_outputs(self,
                queue.handle_unexpected_error,
                args=[work_item, "Mock error message"],
                expected_stdout=expected_stdout.get("handle_unexpected_error", ""),
                expected_stderr=expected_stderr.get("handle_unexpected_error", ""))
