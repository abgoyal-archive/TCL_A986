from google.appengine.ext import webapp
from google.appengine.ext.webapp import template

from model.queues import queues
from model.queuestatus import QueueStatus

class RecentStatus(webapp.RequestHandler):
    def _title_case(self, string):
        words = string.split(" ")
        words = map(lambda word: word.capitalize(), words)
        return " ".join(words)

    def _pretty_queue_name(self, queue_name):
        return self._title_case(queue_name.replace("-", " "))

    # We could change "/" to just redirect to /queue-status/commit-queue in the future
    # at which point we would not need a default value for queue_name here.
    def get(self, queue_name="commit-queue"):
        queue_status = {}
        for queue in queues:
            statuses = QueueStatus.all().filter("queue_name =", queue).order("-date").fetch(6)
            if not statuses:
                continue
            queue_status[queue] = statuses

        template_values = {
            "queue_status" : queue_status,
        }
        self.response.out.write(template.render("templates/recentstatus.html", template_values))
