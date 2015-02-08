from google.appengine.ext import webapp
from google.appengine.ext.webapp import template

from model.attachment import Attachment


class StatusBubble(webapp.RequestHandler):
    def get(self, attachment_id):
        attachment = Attachment(int(attachment_id))

        template_values = {
            "summary" : attachment.summary()
        }
        self.response.out.write(template.render("templates/statusbubble.html", template_values))
