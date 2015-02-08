import webkitpy.steps as steps

from webkitpy.executive import ScriptError
from webkitpy.webkit_logging import log
from webkitpy.scm import CheckoutNeedsUpdate
from webkitpy.queueengine import QueueEngine


class StepSequenceErrorHandler():
    @classmethod
    def handle_script_error(cls, tool, patch, script_error):
        raise NotImplementedError, "subclasses must implement"


class StepSequence(object):
    def __init__(self, steps):
        self._steps = steps or []

    def options(self):
        collected_options = [
            steps.Options.parent_command,
            steps.Options.quiet,
        ]
        for step in self._steps:
            collected_options = collected_options + step.options()
        # Remove duplicates.
        collected_options = sorted(set(collected_options))
        return collected_options

    def _run(self, tool, options, state):
        for step in self._steps:
            step(tool, options).run(state)

    def run_and_handle_errors(self, tool, options, state=None):
        if not state:
            state = {}
        try:
            self._run(tool, options, state)
        except CheckoutNeedsUpdate, e:
            log("Commit failed because the checkout is out of date.  Please update and try again.")
            log("You can pass --no-build to skip building/testing after update if you believe the new commits did not affect the results.")
            QueueEngine.exit_after_handled_error(e)
        except ScriptError, e:
            if not options.quiet:
                log(e.message_with_output())
            if options.parent_command:
                command = tool.command_by_name(options.parent_command)
                command.handle_script_error(tool, state, e)
            QueueEngine.exit_after_handled_error(e)
