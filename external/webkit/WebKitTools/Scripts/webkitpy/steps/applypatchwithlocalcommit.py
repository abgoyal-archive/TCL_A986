from webkitpy.steps.applypatch import ApplyPatch
from webkitpy.steps.options import Options

class ApplyPatchWithLocalCommit(ApplyPatch):
    @classmethod
    def options(cls):
        return [
            Options.local_commit,
        ] + ApplyPatch.options()

    def run(self, state):
        ApplyPatch.run(self, state)
        if self._options.local_commit:
            commit_message = self._tool.scm().commit_message_for_this_commit()
            self._tool.scm().commit_locally_with_message(commit_message.message() or state["patch"].name())
