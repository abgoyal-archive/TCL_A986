from webkitpy.networktransaction import NetworkTransaction
from webkitpy.webkit_logging import log
from mechanize import Browser

# WebKit includes a built copy of BeautifulSoup in Scripts/webkitpy
# so this import should always succeed.
from .BeautifulSoup import BeautifulSoup

import urllib2


class StatusServer:
    default_host = "webkit-commit-queue.appspot.com"

    def __init__(self, host=default_host):
        self.set_host(host)
        self.browser = Browser()

    def set_host(self, host):
        self.host = host
        self.url = "http://%s" % self.host

    def results_url_for_status(self, status_id):
        return "%s/results/%s" % (self.url, status_id)

    def _add_patch(self, patch):
        if not patch:
            return
        if patch.bug_id():
            self.browser["bug_id"] = str(patch.bug_id())
        if patch.id():
            self.browser["patch_id"] = str(patch.id())

    def _add_results_file(self, results_file):
        if not results_file:
            return
        self.browser.add_file(results_file, "text/plain", "results.txt", 'results_file')

    def _post_to_server(self, queue_name, status, patch, results_file):
        if results_file:
            # We might need to re-wind the file if we've already tried to post it.
            results_file.seek(0)

        update_status_url = "%s/update-status" % self.url
        self.browser.open(update_status_url)
        self.browser.select_form(name="update_status")
        self.browser['queue_name'] = queue_name
        self._add_patch(patch)
        self.browser['status'] = status
        self._add_results_file(results_file)
        return self.browser.submit().read() # This is the id of the newly created status object.

    def update_status(self, queue_name, status, patch=None, results_file=None):
        # During unit testing, host is None
        if not self.host:
            return

        log(status)
        return NetworkTransaction().run(lambda: self._post_to_server(queue_name, status, patch, results_file))

    def patch_status(self, queue_name, patch_id):
        update_status_url = "%s/patch-status/%s/%s" % (self.url, queue_name, patch_id)
        try:
            return urllib2.urlopen(update_status_url).read()
        except urllib2.HTTPError, e:
            if e.code == 404:
                return None
            raise e
