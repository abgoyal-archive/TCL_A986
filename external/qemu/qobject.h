
#ifndef QOBJECT_H
#define QOBJECT_H

#include <stddef.h>
#include <assert.h>

typedef enum {
    QTYPE_NONE,
    QTYPE_QINT,
    QTYPE_QSTRING,
    QTYPE_QDICT,
    QTYPE_QLIST,
    QTYPE_QFLOAT,
    QTYPE_QBOOL,
    QTYPE_QERROR,
} qtype_code;

struct QObject;

typedef struct QType {
    qtype_code code;
    void (*destroy)(struct QObject *);
} QType;

typedef struct QObject {
    const QType *type;
    size_t refcnt;
} QObject;

/* Objects definitions must include this */
#define QObject_HEAD  \
    QObject base

/* Get the 'base' part of an object */
#define QOBJECT(obj) (&(obj)->base)

/* High-level interface for qobject_incref() */
#define QINCREF(obj)      \
    qobject_incref(QOBJECT(obj))

/* High-level interface for qobject_decref() */
#define QDECREF(obj)              \
    qobject_decref(QOBJECT(obj))

/* Initialize an object to default values */
#define QOBJECT_INIT(obj, qtype_type)   \
    obj->base.refcnt = 1;               \
    obj->base.type   = qtype_type

static inline void qobject_incref(QObject *obj)
{
    if (obj)
        obj->refcnt++;
}

static inline void qobject_decref(QObject *obj)
{
    if (obj && --obj->refcnt == 0) {
        assert(obj->type != NULL);
        assert(obj->type->destroy != NULL);
        obj->type->destroy(obj);
    }
}

static inline qtype_code qobject_type(const QObject *obj)
{
    assert(obj->type != NULL);
    return obj->type->code;
}

#endif /* QOBJECT_H */
