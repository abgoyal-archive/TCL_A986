from google.appengine.api import users
from google.appengine.ext import webapp, db
from google.appengine.ext.webapp import template

from model.attachment import Attachment
from model.queuestatus import QueueStatus


class UpdateStatus(webapp.RequestHandler):
    def get(self):
        self.response.out.write(template.render("templates/updatestatus.html", None))

    def _int_from_request(self, name):
        string_value = self.request.get(name)
        try:
            int_value = int(string_value)
            return int_value
        except ValueError, TypeError:
            pass
        return None

    def post(self):
        queue_status = QueueStatus()

        if users.get_current_user():
            queue_status.author = users.get_current_user()

        bug_id = self._int_from_request("bug_id")
        patch_id = self._int_from_request("patch_id")
        queue_name = self.request.get("queue_name")
        queue_status.queue_name = queue_name
        queue_status.active_bug_id = bug_id
        queue_status.active_patch_id = patch_id
        queue_status.message = self.request.get("status")
        results_file = self.request.get("results_file")
        queue_status.results_file = db.Blob(str(results_file))
        queue_status.put()
        Attachment.dirty(patch_id)
        self.response.out.write(queue_status.key().id())
