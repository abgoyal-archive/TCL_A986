import StringIO

from webkitpy.steps.abstractstep import AbstractStep
from webkitpy.steps.options import Options


class PostDiff(AbstractStep):
    @classmethod
    def options(cls):
        return [
            Options.description,
            Options.review,
            Options.request_commit,
            Options.open_bug,
        ]

    def run(self, state):
        diff = self.cached_lookup(state, "diff")
        diff_file = StringIO.StringIO(diff) # add_patch_to_bug expects a file-like object
        description = self._options.description or "Patch"
        self._tool.bugs.add_patch_to_bug(state["bug_id"], diff_file, description, mark_for_review=self._options.review, mark_for_commit_queue=self._options.request_commit)
        if self._options.open_bug:
            self._tool.user.open_url(self._tool.bugs.bug_url_for_bug_id(state["bug_id"]))
