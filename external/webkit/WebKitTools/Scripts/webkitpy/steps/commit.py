from webkitpy.steps.abstractstep import AbstractStep


class Commit(AbstractStep):
    def run(self, state):
        commit_message = self._tool.scm().commit_message_for_this_commit()
        state["commit_text"] =  self._tool.scm().commit_with_message(commit_message.message())
