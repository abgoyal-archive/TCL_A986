import StringIO

from webkitpy.steps.abstractstep import AbstractStep


class PostDiffForCommit(AbstractStep):
    def run(self, state):
        self._tool.bugs.add_patch_to_bug(
            state["bug_id"],
            StringIO.StringIO(self.cached_lookup(state, "diff")),
            "Patch for landing",
            mark_for_review=False,
            mark_for_landing=True)
