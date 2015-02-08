from webkitpy.comments import bug_comment_from_commit_text
from webkitpy.steps.abstractstep import AbstractStep
from webkitpy.steps.options import Options
from webkitpy.webkit_logging import log


class CloseBugForLandDiff(AbstractStep):
    @classmethod
    def options(cls):
        return [
            Options.close_bug,
        ]

    def run(self, state):
        comment_text = bug_comment_from_commit_text(self._tool.scm(), state["commit_text"])
        bug_id = state.get("bug_id")
        if not bug_id and state.get("patch"):
            bug_id = state.get("patch").bug_id()

        if bug_id:
            log("Updating bug %s" % bug_id)
            if self._options.close_bug:
                self._tool.bugs.close_bug_as_fixed(bug_id, comment_text)
            else:
                # FIXME: We should a smart way to figure out if the patch is attached
                # to the bug, and if so obsolete it.
                self._tool.bugs.post_comment_to_bug(bug_id, comment_text)
        else:
            log(comment_text)
            log("No bug id provided.")
