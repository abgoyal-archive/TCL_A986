from webkitpy.steps.abstractstep import AbstractStep
from webkitpy.steps.options import Options
from webkitpy.webkit_logging import log


class Update(AbstractStep):
    @classmethod
    def options(cls):
        return [
            Options.update,
            Options.port,
        ]

    def run(self, state):
        if not self._options.update:
            return
        log("Updating working directory")
        self._tool.executive.run_and_throw_if_fail(self.port().update_webkit_command(), quiet=True)
