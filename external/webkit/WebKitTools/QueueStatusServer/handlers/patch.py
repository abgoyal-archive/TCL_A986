from google.appengine.ext import webapp
from google.appengine.ext.webapp import template

from model.queuestatus import QueueStatus


class Patch(webapp.RequestHandler):
    def get(self, attachment_id_string):
        attachment_id = int(attachment_id_string)
        statuses = QueueStatus.all().filter("active_patch_id =", attachment_id).order("-date")

        bug_id = None
        queue_status = {}
        for status in statuses:
            bug_id = status.active_bug_id # Should be the same for every status.
            per_queue_statuses = queue_status.get(status.queue_name, [])
            per_queue_statuses.append(status)
            queue_status[status.queue_name] = per_queue_statuses

        template_values = {
            "attachment_id" : attachment_id,
            "bug_id" : bug_id,
            "queue_status" : queue_status,
        }
        self.response.out.write(template.render("templates/patch.html", template_values))
