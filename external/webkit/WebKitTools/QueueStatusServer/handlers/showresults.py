from google.appengine.ext import webapp

from model.queuestatus import QueueStatus


class ShowResults(webapp.RequestHandler):
    def get(self, status_id):
        status = QueueStatus.get_by_id(int(status_id))
        if not status:
            self.error(404)
            return
        self.response.headers["Content-Type"] = "text/plain; charset=utf-8"
        self.response.out.write(status.results_file)
