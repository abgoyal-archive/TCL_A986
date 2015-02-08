

#ifndef QJSON_H
#define QJSON_H

#include <stdarg.h>
#include "qobject.h"
#include "qstring.h"

QObject *qobject_from_json(const char *string);
QObject *qobject_from_jsonf(const char *string, ...)
    __attribute__((__format__ (__printf__, 1, 2)));
QObject *qobject_from_jsonv(const char *string, va_list *ap);

QString *qobject_to_json(const QObject *obj);

#endif /* QJSON_H */
