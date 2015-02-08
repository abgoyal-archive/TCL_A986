import os

from webkitpy.changelogs import ChangeLog
from webkitpy.grammar import pluralize
from webkitpy.steps.abstractstep import AbstractStep
from webkitpy.steps.options import Options
from webkitpy.webkit_logging import log, error

class UpdateChangeLogsWithReviewer(AbstractStep):
    @classmethod
    def options(cls):
        return [
            Options.reviewer,
        ]

    def _guess_reviewer_from_bug(self, bug_id):
        patches = self._tool.bugs.fetch_bug(bug_id).reviewed_patches()
        if len(patches) != 1:
            log("%s on bug %s, cannot infer reviewer." % (pluralize("reviewed patch", len(patches)), bug_id))
            return None
        patch = patches[0]
        log("Guessing \"%s\" as reviewer from attachment %s on bug %s." % (patch.reviewer().full_name, patch.id(), bug_id))
        return patch.reviewer().full_name

    def run(self, state):
        bug_id = state.get("bug_id")
        if not bug_id and state.get("patch"):
            bug_id = state.get("patch").bug_id()

        reviewer = self._options.reviewer
        if not reviewer:
            if not bug_id:
                log("No bug id provided and --reviewer= not provided.  Not updating ChangeLogs with reviewer.")
                return
            reviewer = self._guess_reviewer_from_bug(bug_id)

        if not reviewer:
            log("Failed to guess reviewer from bug %s and --reviewer= not provided.  Not updating ChangeLogs with reviewer." % bug_id)
            return

        os.chdir(self._tool.scm().checkout_root)
        for changelog_path in self._tool.scm().modified_changelogs():
            ChangeLog(changelog_path).set_reviewer(reviewer)
