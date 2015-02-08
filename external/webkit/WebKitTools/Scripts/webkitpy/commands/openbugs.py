import re
import sys

from webkitpy.multicommandtool import AbstractDeclarativeCommand
from webkitpy.webkit_logging import log


class OpenBugs(AbstractDeclarativeCommand):
    name = "open-bugs"
    help_text = "Finds all bug numbers passed in arguments (or stdin if no args provided) and opens them in a web browser"

    bug_number_regexp = re.compile(r"\b\d{4,6}\b")

    def _open_bugs(self, bug_ids):
        for bug_id in bug_ids:
            bug_url = self.tool.bugs.bug_url_for_bug_id(bug_id)
            self.tool.user.open_url(bug_url)

    # _find_bugs_in_string mostly exists for easy unit testing.
    def _find_bugs_in_string(self, string):
        return self.bug_number_regexp.findall(string)

    def _find_bugs_in_iterable(self, iterable):
        return sum([self._find_bugs_in_string(string) for string in iterable], [])

    def execute(self, options, args, tool):
        if args:
            bug_ids = self._find_bugs_in_iterable(args)
        else:
            # This won't open bugs until stdin is closed but could be made to easily.  That would just make unit testing slightly harder.
            bug_ids = self._find_bugs_in_iterable(sys.stdin)

        log("%s bugs found in input." % len(bug_ids))

        self._open_bugs(bug_ids)
