#!/usr/bin/env python
class PersistentPatchCollectionDelegate:
    def collection_name(self):
        raise NotImplementedError, "subclasses must implement"

    def fetch_potential_patch_ids(self):
        raise NotImplementedError, "subclasses must implement"

    def status_server(self):
        raise NotImplementedError, "subclasses must implement"

    def is_terminal_status(self, status):
        raise NotImplementedError, "subclasses must implement"


class PersistentPatchCollection:
    def __init__(self, delegate):
        self._delegate = delegate
        self._name = self._delegate.collection_name()
        self._status = self._delegate.status_server()
        self._status_cache = {}

    def _cached_status(self, patch_id):
        cached = self._status_cache.get(patch_id)
        if cached:
            return cached
        status = self._status.patch_status(self._name, patch_id)
        if status and self._delegate.is_terminal_status(status):
            self._status_cache[patch_id] = status
        return status

    def next(self):
        patch_ids = self._delegate.fetch_potential_patch_ids()
        for patch_id in patch_ids:
            status = self._cached_status(patch_id)
            if not status or not self._delegate.is_terminal_status(status):
                return patch_id
