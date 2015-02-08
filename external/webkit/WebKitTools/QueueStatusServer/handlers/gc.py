from google.appengine.ext import webapp

from model.queuestatus import QueueStatus


class GC(webapp.RequestHandler):
    def get(self):
        statuses = QueueStatus.all().order("-date")
        seen_queues = set()
        for status in statuses:
            if status.active_patch_id or status.active_bug_id:
                continue
            if status.queue_name in seen_queues:
                status.delete()
            seen_queues.add(status.queue_name)
        self.response.out.write("Done!")
