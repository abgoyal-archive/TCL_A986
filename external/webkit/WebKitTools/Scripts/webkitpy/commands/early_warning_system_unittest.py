import os

from webkitpy.commands.early_warning_system import *
from webkitpy.commands.queuestest import QueuesTest
from webkitpy.mock import Mock

class EarlyWarningSytemTest(QueuesTest):
    def test_chromium_ews(self):
        expected_stderr = {
            "begin_work_queue" : "CAUTION: chromium-ews will discard all local changes in \"%s\"\nRunning WebKit chromium-ews.\n" % os.getcwd(),
            "handle_unexpected_error" : "Mock error message\n",
        }
        self.assert_queue_outputs(ChromiumEWS(), expected_stderr=expected_stderr)

    def test_qt_ews(self):
        expected_stderr = {
            "begin_work_queue" : "CAUTION: qt-ews will discard all local changes in \"%s\"\nRunning WebKit qt-ews.\n" % os.getcwd(),
            "handle_unexpected_error" : "Mock error message\n",
        }
        self.assert_queue_outputs(QtEWS(), expected_stderr=expected_stderr)

    def test_gtk_ews(self):
        expected_stderr = {
            "begin_work_queue" : "CAUTION: gtk-ews will discard all local changes in \"%s\"\nRunning WebKit gtk-ews.\n" % os.getcwd(),
            "handle_unexpected_error" : "Mock error message\n",
        }
        self.assert_queue_outputs(GtkEWS(), expected_stderr=expected_stderr)

    def test_mac_ews(self):
        expected_stderr = {
            "begin_work_queue" : "CAUTION: mac-ews will discard all local changes in \"%s\"\nRunning WebKit mac-ews.\n" % os.getcwd(),
            "handle_unexpected_error" : "Mock error message\n",
        }
        self.assert_queue_outputs(MacEWS(), expected_stderr=expected_stderr)
