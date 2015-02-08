from webkitpy.grammar import pluralize
from webkitpy.steps.abstractstep import AbstractStep
from webkitpy.steps.options import Options
from webkitpy.webkit_logging import log


class ObsoletePatches(AbstractStep):
    @classmethod
    def options(cls):
        return [
            Options.obsolete_patches,
        ]

    def run(self, state):
        if not self._options.obsolete_patches:
            return
        bug_id = state["bug_id"]
        patches = self._tool.bugs.fetch_bug(bug_id).patches()
        if not patches:
            return
        log("Obsoleting %s on bug %s" % (pluralize("old patch", len(patches)), bug_id))
        for patch in patches:
            self._tool.bugs.obsolete_attachment(patch.id())
