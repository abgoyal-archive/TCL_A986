
#include "qlist.h"
#include "qobject.h"
#include "qemu-queue.h"
#include "qemu-common.h"

static void qlist_destroy_obj(QObject *obj);

static const QType qlist_type = {
    .code = QTYPE_QLIST,
    .destroy = qlist_destroy_obj,
};
 
QList *qlist_new(void)
{
    QList *qlist;

    qlist = qemu_malloc(sizeof(*qlist));
    QTAILQ_INIT(&qlist->head);
    QOBJECT_INIT(qlist, &qlist_type);

    return qlist;
}

static void qlist_copy_elem(QObject *obj, void *opaque)
{
    QList *dst = opaque;

    qobject_incref(obj);
    qlist_append_obj(dst, obj);
}

QList *qlist_copy(QList *src)
{
    QList *dst = qlist_new();

    qlist_iter(src, qlist_copy_elem, dst);

    return dst;
}

void qlist_append_obj(QList *qlist, QObject *value)
{
    QListEntry *entry;

    entry = qemu_malloc(sizeof(*entry));
    entry->value = value;

    QTAILQ_INSERT_TAIL(&qlist->head, entry, next);
}

void qlist_iter(const QList *qlist,
                void (*iter)(QObject *obj, void *opaque), void *opaque)
{
    QListEntry *entry;

    QTAILQ_FOREACH(entry, &qlist->head, next)
        iter(entry->value, opaque);
}

QObject *qlist_pop(QList *qlist)
{
    QListEntry *entry;
    QObject *ret;

    if (qlist == NULL || QTAILQ_EMPTY(&qlist->head)) {
        return NULL;
    }

    entry = QTAILQ_FIRST(&qlist->head);
    QTAILQ_REMOVE(&qlist->head, entry, next);

    ret = entry->value;
    qemu_free(entry);

    return ret;
}

QObject *qlist_peek(QList *qlist)
{
    QListEntry *entry;
    QObject *ret;

    if (qlist == NULL || QTAILQ_EMPTY(&qlist->head)) {
        return NULL;
    }

    entry = QTAILQ_FIRST(&qlist->head);

    ret = entry->value;

    return ret;
}

int qlist_empty(const QList *qlist)
{
    return QTAILQ_EMPTY(&qlist->head);
}

QList *qobject_to_qlist(const QObject *obj)
{
    if (qobject_type(obj) != QTYPE_QLIST) {
        return NULL;
    }

    return container_of(obj, QList, base);
}

static void qlist_destroy_obj(QObject *obj)
{
    QList *qlist;
    QListEntry *entry, *next_entry;

    assert(obj != NULL);
    qlist = qobject_to_qlist(obj);

    QTAILQ_FOREACH_SAFE(entry, &qlist->head, next, next_entry) {
        QTAILQ_REMOVE(&qlist->head, entry, next);
        qobject_decref(entry->value);
        qemu_free(entry);
    }

    qemu_free(qlist);
}
