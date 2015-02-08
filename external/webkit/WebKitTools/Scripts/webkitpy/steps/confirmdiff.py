from webkitpy.steps.abstractstep import AbstractStep
from webkitpy.steps.options import Options
from webkitpy.webkit_logging import error


class ConfirmDiff(AbstractStep):
    @classmethod
    def options(cls):
        return [
            Options.confirm,
        ]

    def run(self, state):
        if not self._options.confirm:
            return
        diff = self.cached_lookup(state, "diff")
        self._tool.user.page(diff)
        if not self._tool.user.confirm("Was that diff correct?"):
            exit(1)
