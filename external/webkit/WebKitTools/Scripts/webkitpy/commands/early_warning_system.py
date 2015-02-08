#!/usr/bin/env python
from StringIO import StringIO

from webkitpy.commands.queues import AbstractReviewQueue
from webkitpy.committers import CommitterList
from webkitpy.executive import ScriptError
from webkitpy.webkitport import WebKitPort
from webkitpy.queueengine import QueueEngine


class AbstractEarlyWarningSystem(AbstractReviewQueue):
    _build_style = "release"

    def __init__(self):
        AbstractReviewQueue.__init__(self)
        self.port = WebKitPort.port(self.port_name)

    def should_proceed_with_work_item(self, patch):
        try:
            self.run_webkit_patch([
                "build",
                self.port.flag(),
                "--build-style=%s" % self._build_style,
                "--force-clean",
                "--quiet"])
            self._update_status("Building", patch)
        except ScriptError, e:
            self._update_status("Unable to perform a build")
            return False
        return True

    def _review_patch(self, patch):
        self.run_webkit_patch([
            "build-attachment",
            self.port.flag(),
            "--build-style=%s" % self._build_style,
            "--force-clean",
            "--quiet",
            "--non-interactive",
            "--parent-command=%s" % self.name,
            "--no-update",
            patch.id()])

    @classmethod
    def handle_script_error(cls, tool, state, script_error):
        is_svn_apply = script_error.command_name() == "svn-apply"
        status_id = cls._update_status_for_script_error(tool, state, script_error, is_error=is_svn_apply)
        if is_svn_apply:
            QueueEngine.exit_after_handled_error(script_error)
        results_link = tool.status_server.results_url_for_status(status_id)
        message = "Attachment %s did not build on %s:\nBuild output: %s" % (state["patch"].id(), cls.port_name, results_link)
        tool.bugs.post_comment_to_bug(state["patch"].bug_id(), message, cc=cls.watchers)
        exit(1)


class GtkEWS(AbstractEarlyWarningSystem):
    name = "gtk-ews"
    port_name = "gtk"
    watchers = AbstractEarlyWarningSystem.watchers + [
        "gns@gnome.org",
        "xan.lopez@gmail.com",
    ]


class QtEWS(AbstractEarlyWarningSystem):
    name = "qt-ews"
    port_name = "qt"


class ChromiumEWS(AbstractEarlyWarningSystem):
    name = "chromium-ews"
    port_name = "chromium"
    watchers = AbstractEarlyWarningSystem.watchers + [
        "dglazkov@chromium.org",
    ]


# For platforms that we can't run inside a VM (like Mac OS X), we require
# patches to be uploaded by committers, who are generally trustworthy folk. :)
class AbstractCommitterOnlyEWS(AbstractEarlyWarningSystem):
    def __init__(self, committers=CommitterList()):
        AbstractEarlyWarningSystem.__init__(self)
        self._committers = committers

    def process_work_item(self, patch):
        if not self._committers.committer_by_email(patch.attacher_email()):
            self._did_error(patch, "%s cannot process patches from non-committers :(" % self.name)
            return
        AbstractEarlyWarningSystem.process_work_item(self, patch)


class MacEWS(AbstractCommitterOnlyEWS):
    name = "mac-ews"
    port_name = "mac"
