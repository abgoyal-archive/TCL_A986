from webkitpy.steps.abstractstep import AbstractStep
from webkitpy.steps.options import Options


class CreateBug(AbstractStep):
    @classmethod
    def options(cls):
        return [
            Options.cc,
            Options.component,
        ]

    def run(self, state):
        # No need to create a bug if we already have one.
        if state.get("bug_id"):
            return
        state["bug_id"] = self._tool.bugs.create_bug(state["bug_title"], state["bug_description"], component=self._options.component, cc=self._options.cc)
