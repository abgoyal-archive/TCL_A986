import os

from webkitpy.steps.abstractstep import AbstractStep


class EditChangeLog(AbstractStep):
    def run(self, state):
        os.chdir(self._tool.scm().checkout_root)
        self._tool.user.edit(self.cached_lookup(state, "changelogs"))
