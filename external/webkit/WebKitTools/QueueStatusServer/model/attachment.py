import re

from google.appengine.api import memcache

from model.queues import queues
from model.queuestatus import QueueStatus


class Attachment(object):
    @classmethod
    def dirty(cls, attachment_id):
        memcache.delete(str(attachment_id), namespace="attachment-summary")

    @classmethod
    def recent(cls, limit=1):
        statuses = QueueStatus.all().order("-date")
        # Notice that we use both a set and a list here to keep the -date ordering.
        ids = []
        visited_ids = set()
        for status in statuses:
            attachment_id = status.active_patch_id
            if not attachment_id:
                continue
            if attachment_id in visited_ids:
                continue
            visited_ids.add(attachment_id)
            ids.append(attachment_id)
            if len(visited_ids) >= limit:
                break
        return map(cls, ids)

    def __init__(self, attachment_id):
        self.id = attachment_id
        self._summary = None

    def summary(self):
        if self._summary:
            return self._summary
        self._summary = memcache.get(str(self.id), namespace="attachment-summary")
        if self._summary:
            return self._summary
        self._summary = self._fetch_summary()
        memcache.set(str(self.id), self._summary, namespace="attachment-summary")
        return self._summary

    def _dash_to_underscore(self, dashed_name):
        regexp = re.compile("-")
        return regexp.sub("_", dashed_name)

    def _state_from_status(self, status):
        table = {
            "Pass" : "pass",
            "Fail" : "fail",
        }
        state = table.get(status.message)
        if state:
            return state
        if status.message.startswith("Error:"):
            return "error"
        if status:
            return "pending"
        return None

    def _fetch_summary(self):
        summary = { "attachment_id" : self.id }

        first_status = QueueStatus.all().filter('active_patch_id =', self.id).get()
        if not first_status:
            # We don't have any record of this attachment.
            return summary
        summary["bug_id"] = first_status.active_bug_id

        for queue in queues:
            summary[queue] = None
            status = QueueStatus.all().filter('queue_name =', queue).filter('active_patch_id =', self.id).order('-date').get()
            if status:
                summary[self._dash_to_underscore(queue)] = {
                    "state" : self._state_from_status(status),
                    "status" : status,
                }
        return summary
