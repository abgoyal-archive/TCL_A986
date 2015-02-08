from google.appengine.ext import webapp

from model.queuestatus import QueueStatus


class PatchStatus(webapp.RequestHandler):
    def get(self, queue_name, attachment_id):
        statuses = QueueStatus.all().filter('queue_name =', queue_name).filter('active_patch_id =', int(attachment_id)).order('-date').fetch(1)
        if not statuses:
            self.error(404)
            return
        self.response.out.write(statuses[0].message)
