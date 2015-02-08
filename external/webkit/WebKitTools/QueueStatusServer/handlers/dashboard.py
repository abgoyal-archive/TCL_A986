from google.appengine.ext import webapp
from google.appengine.ext.webapp import template

from model.attachment import Attachment

class Dashboard(webapp.RequestHandler):
    def get(self):
        attachments = Attachment.recent(limit=25)

        template_values = {
            "summaries" : [attachment.summary() for attachment in attachments],
        }
        self.response.out.write(template.render("templates/dashboard.html", template_values))
