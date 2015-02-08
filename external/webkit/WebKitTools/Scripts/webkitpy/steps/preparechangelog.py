import os

from webkitpy.executive import ScriptError
from webkitpy.steps.abstractstep import AbstractStep
from webkitpy.steps.options import Options
from webkitpy.webkit_logging import error


class PrepareChangeLog(AbstractStep):
    @classmethod
    def options(cls):
        return [
            Options.port,
            Options.quiet,
            Options.email,
        ]

    def run(self, state):
        if self.cached_lookup(state, "changelogs"):
            return
        os.chdir(self._tool.scm().checkout_root)
        args = [self.port().script_path("prepare-ChangeLog")]
        if state["bug_id"]:
            args.append("--bug=%s" % state["bug_id"])
        if self._options.email:
            args.append("--email=%s" % self._options.email)
        try:
            self._tool.executive.run_and_throw_if_fail(args, self._options.quiet)
        except ScriptError, e:
            error("Unable to prepare ChangeLogs.")
        state["diff"] = None # We've changed the diff
