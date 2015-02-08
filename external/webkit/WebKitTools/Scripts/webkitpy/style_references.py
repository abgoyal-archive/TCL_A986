"""References to non-style modules used by the style package."""

# This module is a simple facade to the functionality used by the
# style package that comes from WebKit modules outside the style
# package.
#
# With this module, the only intra-package references (i.e.
# references to webkitpy modules outside the style folder) that
# the style package needs to make are relative references to
# this module. For example--
#
# > from .. style_references import parse_patch
#
# Similarly, people maintaining non-style code are not beholden
# to the contents of the style package when refactoring or
# otherwise changing non-style code. They only have to be aware
# of this module.

import os

from diff_parser import DiffParser
from scm import detect_scm_system


def parse_patch(patch_string):

    """Parse a patch string and return the affected files."""

    patch = DiffParser(patch_string.splitlines())
    return patch.files


class SimpleScm(object):

    """Simple facade to SCM for use by style package."""

    def __init__(self):
        cwd = os.path.abspath('.')
        self._scm = detect_scm_system(cwd)

    def checkout_root(self):
        """Return the source control root as an absolute path."""
        return self._scm.checkout_root

    def create_patch(self):
        return self._scm.create_patch()

    def create_patch_since_local_commit(self, commit):
        return self._scm.create_patch_since_local_commit(commit)

