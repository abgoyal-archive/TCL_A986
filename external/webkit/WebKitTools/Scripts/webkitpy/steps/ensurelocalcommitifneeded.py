from webkitpy.steps.abstractstep import AbstractStep
from webkitpy.steps.options import Options
from webkitpy.webkit_logging import error


class EnsureLocalCommitIfNeeded(AbstractStep):
    @classmethod
    def options(cls):
        return [
            Options.local_commit,
        ]

    def run(self, state):
        if self._options.local_commit and not self._tool.scm().supports_local_commits():
            error("--local-commit passed, but %s does not support local commits" % self._tool.scm.display_name())
