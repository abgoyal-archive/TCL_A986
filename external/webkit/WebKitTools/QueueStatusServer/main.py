from google.appengine.dist import use_library
use_library('django', '1.1')

from google.appengine.ext import webapp
from google.appengine.ext.webapp.util import run_wsgi_app

from handlers.dashboard import Dashboard
from handlers.gc import GC
from handlers.patch import Patch
from handlers.patchstatus import PatchStatus
from handlers.recentstatus import RecentStatus
from handlers.showresults import ShowResults
from handlers.statusbubble import StatusBubble
from handlers.updatestatus import UpdateStatus

webapp.template.register_template_library('filters.webkit_extras')

routes = [
    ('/', RecentStatus),
    ('/dashboard', Dashboard),
    ('/gc', GC),
    (r'/patch-status/(.*)/(.*)', PatchStatus),
    (r'/patch/(.*)', Patch),
    (r'/results/(.*)', ShowResults),
    (r'/status-bubble/(.*)', StatusBubble),
    (r'/queue-status/(.*)', RecentStatus),
    ('/update-status', UpdateStatus),
]

application = webapp.WSGIApplication(routes, debug=True)

def main():
    run_wsgi_app(application)

if __name__ == "__main__":
    main()
