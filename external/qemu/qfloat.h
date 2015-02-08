

#ifndef QFLOAT_H
#define QFLOAT_H

#include <stdint.h>
#include "qobject.h"

typedef struct QFloat {
    QObject_HEAD;
    double value;
} QFloat;

QFloat *qfloat_from_double(double value);
double qfloat_get_double(const QFloat *qi);
QFloat *qobject_to_qfloat(const QObject *obj);

#endif /* QFLOAT_H */
