import unittest

from webkitpy.steps.update import Update
from webkitpy.steps.promptforbugortitle import PromptForBugOrTitle
from webkitpy.mock_bugzillatool import MockBugzillaTool
from webkitpy.outputcapture import OutputCapture
from webkitpy.mock import Mock


class StepsTest(unittest.TestCase):
    def _run_step(self, step, tool=None, options=None, state=None):
        if not tool:
            tool = MockBugzillaTool()
        if not options:
            options = Mock()
        if not state:
            state = {}
        step(tool, options).run(state)

    def test_update_step(self):
        options = Mock()
        options.update = True
        self._run_step(Update, options)

    def test_prompt_for_bug_or_title_step(self):
        tool = MockBugzillaTool()
        tool.user.prompt = lambda message: 42
        self._run_step(PromptForBugOrTitle, tool=tool)
