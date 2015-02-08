from webkitpy.comments import bug_comment_from_commit_text
from webkitpy.steps.abstractstep import AbstractStep


class ClosePatch(AbstractStep):
    def run(self, state):
        comment_text = bug_comment_from_commit_text(self._tool.scm(), state["commit_text"])
        self._tool.bugs.clear_attachment_flags(state["patch"].id(), comment_text)
