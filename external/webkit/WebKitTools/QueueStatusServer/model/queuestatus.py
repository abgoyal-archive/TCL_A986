from google.appengine.ext import db

class QueueStatus(db.Model):
    author = db.UserProperty()
    queue_name = db.StringProperty()
    active_bug_id = db.IntegerProperty()
    active_patch_id = db.IntegerProperty()
    message = db.StringProperty(multiline=True)
    date = db.DateTimeProperty(auto_now_add=True)
    results_file = db.BlobProperty()
