from webkitpy.steps.abstractstep import AbstractStep


class PromptForBugOrTitle(AbstractStep):
    def run(self, state):
        # No need to prompt if we alrady have the bug_id.
        if state.get("bug_id"):
            return
        user_response = self._tool.user.prompt("Please enter a bug number or a title for a new bug:\n")
        # If the user responds with a number, we assume it's bug number.
        # Otherwise we assume it's a bug subject.
        try:
            state["bug_id"] = int(user_response)
        except ValueError, TypeError:
            state["bug_title"] = user_response
            # FIXME: This is kind of a lame description.
            state["bug_description"] = user_response
