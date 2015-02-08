from webkitpy.steps.abstractstep import AbstractStep


class RevertRevision(AbstractStep):
    def run(self, state):
        self._tool.scm().apply_reverse_diff(state["revision"])
