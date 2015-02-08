import os

from webkitpy.changelogs import ChangeLog
from webkitpy.steps.abstractstep import AbstractStep


class PrepareChangeLogForRevert(AbstractStep):
    def run(self, state):
        # First, discard the ChangeLog changes from the rollout.
        os.chdir(self._tool.scm().checkout_root)
        changelog_paths = self._tool.scm().modified_changelogs()
        self._tool.scm().revert_files(changelog_paths)

        # Second, make new ChangeLog entries for this rollout.
        # This could move to prepare-ChangeLog by adding a --revert= option.
        self._run_script("prepare-ChangeLog")
        bug_url = self._tool.bugs.bug_url_for_bug_id(state["bug_id"]) if state["bug_id"] else None
        for changelog_path in changelog_paths:
            # FIXME: Seems we should prepare the message outside of changelogs.py and then just pass in
            # text that we want to use to replace the reviewed by line.
            ChangeLog(changelog_path).update_for_revert(state["revision"], state["reason"], bug_url)
