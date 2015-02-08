from webkitpy.steps.abstractstep import AbstractStep
from webkitpy.steps.options import Options
from webkitpy.webkit_logging import log


class CloseBug(AbstractStep):
    @classmethod
    def options(cls):
        return [
            Options.close_bug,
        ]

    def run(self, state):
        if not self._options.close_bug:
            return
        # Check to make sure there are no r? or r+ patches on the bug before closing.
        # Assume that r- patches are just previous patches someone forgot to obsolete.
        patches = self._tool.bugs.fetch_bug(state["patch"].bug_id()).patches()
        for patch in patches:
            if patch.review() == "?" or patch.review() == "+":
                log("Not closing bug %s as attachment %s has review=%s.  Assuming there are more patches to land from this bug." % (patch.bug_id(), patch.id(), patch.review()))
                return
        self._tool.bugs.close_bug_as_fixed(state["patch"].bug_id(), "All reviewed patches have been landed.  Closing bug.")
