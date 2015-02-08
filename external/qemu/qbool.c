

#include "qbool.h"
#include "qobject.h"
#include "qemu-common.h"

static void qbool_destroy_obj(QObject *obj);

static const QType qbool_type = {
    .code = QTYPE_QBOOL,
    .destroy = qbool_destroy_obj,
};

QBool *qbool_from_int(int value)
{
    QBool *qb;

    qb = qemu_malloc(sizeof(*qb));
    qb->value = value;
    QOBJECT_INIT(qb, &qbool_type);

    return qb;
}

int qbool_get_int(const QBool *qb)
{
    return qb->value;
}

QBool *qobject_to_qbool(const QObject *obj)
{
    if (qobject_type(obj) != QTYPE_QBOOL)
        return NULL;

    return container_of(obj, QBool, base);
}

static void qbool_destroy_obj(QObject *obj)
{
    assert(obj != NULL);
    qemu_free(qobject_to_qbool(obj));
}
