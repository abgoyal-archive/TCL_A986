from webkitpy.comments import bug_comment_from_commit_text
from webkitpy.steps.build import Build
from webkitpy.steps.commit import Commit
from webkitpy.steps.metastep import MetaStep
from webkitpy.steps.options import Options
from webkitpy.webkit_logging import log


class CompleteRollout(MetaStep):
    substeps = [
        Build,
        Commit,
    ]

    @classmethod
    def options(cls):
        collected_options = cls._collect_options_from_steps(cls.substeps)
        collected_options.append(Options.complete_rollout)
        return collected_options

    def run(self, state):
        bug_id = state["bug_id"]
        # FIXME: Fully automated rollout is not 100% idiot-proof yet, so for now just log with instructions on how to complete the rollout.
        # Once we trust rollout we will remove this option.
        if not self._options.complete_rollout:
            log("\nNOTE: Rollout support is experimental.\nPlease verify the rollout diff and use \"webkit-patch land %s\" to commit the rollout." % bug_id)
            return

        MetaStep.run(self, state)

        commit_comment = bug_comment_from_commit_text(self._tool.scm(), state["commit_text"])
        comment_text = "Reverted r%s for reason:\n\n%s\n\n%s" % (state["revision"], state["reason"], commit_comment)

        if not bug_id:
            log(comment_text)
            log("No bugs were updated.")
            return
        self._tool.bugs.reopen_bug(bug_id, comment_text)
