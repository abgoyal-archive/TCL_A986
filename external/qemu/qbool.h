

#ifndef QBOOL_H
#define QBOOL_H

#include <stdint.h>
#include "qobject.h"

typedef struct QBool {
    QObject_HEAD;
    int value;
} QBool;

QBool *qbool_from_int(int value);
int qbool_get_int(const QBool *qb);
QBool *qobject_to_qbool(const QObject *obj);

#endif /* QBOOL_H */
