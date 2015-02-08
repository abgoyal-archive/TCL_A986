

#include "qint.h"
#include "qfloat.h"
#include "qdict.h"
#include "qbool.h"
#include "qstring.h"
#include "qobject.h"
#include "qemu-queue.h"
#include "qemu-common.h"

static void qdict_destroy_obj(QObject *obj);

static const QType qdict_type = {
    .code = QTYPE_QDICT,
    .destroy = qdict_destroy_obj,
};

QDict *qdict_new(void)
{
    QDict *qdict;

    qdict = qemu_mallocz(sizeof(*qdict));
    QOBJECT_INIT(qdict, &qdict_type);

    return qdict;
}

QDict *qobject_to_qdict(const QObject *obj)
{
    if (qobject_type(obj) != QTYPE_QDICT)
        return NULL;

    return container_of(obj, QDict, base);
}

static unsigned int tdb_hash(const char *name)
{
    unsigned value;	/* Used to compute the hash value.  */
    unsigned   i;	/* Used to cycle through random values. */

    /* Set the initial value from the key size. */
    for (value = 0x238F13AF * strlen(name), i=0; name[i]; i++)
        value = (value + (((const unsigned char *)name)[i] << (i*5 % 24)));

    return (1103515243 * value + 12345);
}

static QDictEntry *alloc_entry(const char *key, QObject *value)
{
    QDictEntry *entry;

    entry = qemu_mallocz(sizeof(*entry));
    entry->key = qemu_strdup(key);
    entry->value = value;

    return entry;
}

static QDictEntry *qdict_find(const QDict *qdict,
                              const char *key, unsigned int hash)
{
    QDictEntry *entry;

    QLIST_FOREACH(entry, &qdict->table[hash], next)
        if (!strcmp(entry->key, key))
            return entry;

    return NULL;
}

void qdict_put_obj(QDict *qdict, const char *key, QObject *value)
{
    unsigned int hash;
    QDictEntry *entry;

    hash = tdb_hash(key) % QDICT_HASH_SIZE;
    entry = qdict_find(qdict, key, hash);
    if (entry) {
        /* replace key's value */
        qobject_decref(entry->value);
        entry->value = value;
    } else {
        /* allocate a new entry */
        entry = alloc_entry(key, value);
        QLIST_INSERT_HEAD(&qdict->table[hash], entry, next);
        qdict->size++;
    }
}

QObject *qdict_get(const QDict *qdict, const char *key)
{
    QDictEntry *entry;

    entry = qdict_find(qdict, key, tdb_hash(key) % QDICT_HASH_SIZE);
    return (entry == NULL ? NULL : entry->value);
}

int qdict_haskey(const QDict *qdict, const char *key)
{
    unsigned int hash = tdb_hash(key) % QDICT_HASH_SIZE;
    return (qdict_find(qdict, key, hash) == NULL ? 0 : 1);
}

size_t qdict_size(const QDict *qdict)
{
    return qdict->size;
}

static QObject *qdict_get_obj(const QDict *qdict, const char *key,
                              qtype_code type)
{
    QObject *obj;

    obj = qdict_get(qdict, key);
    assert(obj != NULL);
    assert(qobject_type(obj) == type);

    return obj;
}

double qdict_get_double(const QDict *qdict, const char *key)
{
    QObject *obj = qdict_get(qdict, key);

    assert(obj);
    switch (qobject_type(obj)) {
    case QTYPE_QFLOAT:
        return qfloat_get_double(qobject_to_qfloat(obj));
    case QTYPE_QINT:
        return qint_get_int(qobject_to_qint(obj));
    default:
        assert(0);
    }
}

int64_t qdict_get_int(const QDict *qdict, const char *key)
{
    QObject *obj = qdict_get_obj(qdict, key, QTYPE_QINT);
    return qint_get_int(qobject_to_qint(obj));
}

int qdict_get_bool(const QDict *qdict, const char *key)
{
    QObject *obj = qdict_get_obj(qdict, key, QTYPE_QBOOL);
    return qbool_get_int(qobject_to_qbool(obj));
}

QList *qdict_get_qlist(const QDict *qdict, const char *key)
{
    return qobject_to_qlist(qdict_get_obj(qdict, key, QTYPE_QLIST));
}

QDict *qdict_get_qdict(const QDict *qdict, const char *key)
{
    return qobject_to_qdict(qdict_get_obj(qdict, key, QTYPE_QDICT));
}

const char *qdict_get_str(const QDict *qdict, const char *key)
{
    QObject *obj = qdict_get_obj(qdict, key, QTYPE_QSTRING);
    return qstring_get_str(qobject_to_qstring(obj));
}

int64_t qdict_get_try_int(const QDict *qdict, const char *key,
                          int64_t err_value)
{
    QObject *obj;

    obj = qdict_get(qdict, key);
    if (!obj || qobject_type(obj) != QTYPE_QINT)
        return err_value;

    return qint_get_int(qobject_to_qint(obj));
}

const char *qdict_get_try_str(const QDict *qdict, const char *key)
{
    QObject *obj;

    obj = qdict_get(qdict, key);
    if (!obj || qobject_type(obj) != QTYPE_QSTRING)
        return NULL;

    return qstring_get_str(qobject_to_qstring(obj));
}

void qdict_iter(const QDict *qdict,
                void (*iter)(const char *key, QObject *obj, void *opaque),
                void *opaque)
{
    int i;
    QDictEntry *entry;

    for (i = 0; i < QDICT_HASH_SIZE; i++) {
        QLIST_FOREACH(entry, &qdict->table[i], next)
            iter(entry->key, entry->value, opaque);
    }
}

static void qentry_destroy(QDictEntry *e)
{
    assert(e != NULL);
    assert(e->key != NULL);
    assert(e->value != NULL);

    qobject_decref(e->value);
    qemu_free(e->key);
    qemu_free(e);
}

void qdict_del(QDict *qdict, const char *key)
{
    QDictEntry *entry;

    entry = qdict_find(qdict, key, tdb_hash(key) % QDICT_HASH_SIZE);
    if (entry) {
        QLIST_REMOVE(entry, next);
        qentry_destroy(entry);
        qdict->size--;
    }
}

static void qdict_destroy_obj(QObject *obj)
{
    int i;
    QDict *qdict;

    assert(obj != NULL);
    qdict = qobject_to_qdict(obj);

    for (i = 0; i < QDICT_HASH_SIZE; i++) {
        QDictEntry *entry = QLIST_FIRST(&qdict->table[i]);
        while (entry) {
            QDictEntry *tmp = QLIST_NEXT(entry, next);
            QLIST_REMOVE(entry, next);
            qentry_destroy(entry);
            entry = tmp;
        }
    }

    qemu_free(qdict);
}
