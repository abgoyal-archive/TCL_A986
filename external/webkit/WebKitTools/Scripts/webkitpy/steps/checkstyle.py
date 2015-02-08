import os

from webkitpy.executive import ScriptError
from webkitpy.steps.abstractstep import AbstractStep
from webkitpy.steps.options import Options
from webkitpy.webkit_logging import error

class CheckStyle(AbstractStep):
    @classmethod
    def options(cls):
        return [
            Options.non_interactive,
            Options.check_style,
        ]

    def run(self, state):
        if not self._options.check_style:
            return
        os.chdir(self._tool.scm().checkout_root)
        try:
            self._run_script("check-webkit-style")
        except ScriptError, e:
            if self._options.non_interactive:
                # We need to re-raise the exception here to have the
                # style-queue do the right thing.
                raise e
            if not self._tool.user.confirm("Are you sure you want to continue?"):
                exit(1)
