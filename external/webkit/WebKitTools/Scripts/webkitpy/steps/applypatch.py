from webkitpy.steps.abstractstep import AbstractStep
from webkitpy.steps.options import Options
from webkitpy.webkit_logging import log

class ApplyPatch(AbstractStep):
    @classmethod
    def options(cls):
        return [
            Options.non_interactive,
        ]

    def run(self, state):
        log("Processing patch %s from bug %s." % (state["patch"].id(), state["patch"].bug_id()))
        self._tool.scm().apply_patch(state["patch"], force=self._options.non_interactive)
