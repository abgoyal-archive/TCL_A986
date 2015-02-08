import os

from webkitpy.steps.abstractstep import AbstractStep
from webkitpy.steps.options import Options


class CleanWorkingDirectory(AbstractStep):
    def __init__(self, tool, options, allow_local_commits=False):
        AbstractStep.__init__(self, tool, options)
        self._allow_local_commits = allow_local_commits

    @classmethod
    def options(cls):
        return [
            Options.force_clean,
            Options.clean,
        ]

    def run(self, state):
        os.chdir(self._tool.scm().checkout_root)
        if not self._allow_local_commits:
            self._tool.scm().ensure_no_local_commits(self._options.force_clean)
        if self._options.clean:
            self._tool.scm().ensure_clean_working_directory(force_clean=self._options.force_clean)
