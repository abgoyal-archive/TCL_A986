
#include "qobject.h"
#include "qstring.h"
#include "qemu-common.h"

static void qstring_destroy_obj(QObject *obj);

static const QType qstring_type = {
    .code = QTYPE_QSTRING,
    .destroy = qstring_destroy_obj,
};

QString *qstring_new(void)
{
    return qstring_from_str("");
}

QString *qstring_from_substr(const char *str, int start, int end)
{
    QString *qstring;

    qstring = qemu_malloc(sizeof(*qstring));

    qstring->length = end - start + 1;
    qstring->capacity = qstring->length;

    qstring->string = qemu_malloc(qstring->capacity + 1);
    memcpy(qstring->string, str + start, qstring->length);
    qstring->string[qstring->length] = 0;

    QOBJECT_INIT(qstring, &qstring_type);

    return qstring;
}

QString *qstring_from_str(const char *str)
{
    return qstring_from_substr(str, 0, strlen(str) - 1);
}

static void capacity_increase(QString *qstring, size_t len)
{
    if (qstring->capacity < (qstring->length + len)) {
        qstring->capacity += len;
        qstring->capacity *= 2; /* use exponential growth */

        qstring->string = qemu_realloc(qstring->string, qstring->capacity + 1);
    }
}

void qstring_append(QString *qstring, const char *str)
{
    size_t len = strlen(str);

    capacity_increase(qstring, len);
    memcpy(qstring->string + qstring->length, str, len);
    qstring->length += len;
    qstring->string[qstring->length] = 0;
}

void qstring_append_int(QString *qstring, int64_t value)
{
    char num[32];

    snprintf(num, sizeof(num), "%" PRId64, value);
    qstring_append(qstring, num);
}

void qstring_append_chr(QString *qstring, int c)
{
    capacity_increase(qstring, 1);
    qstring->string[qstring->length++] = c;
    qstring->string[qstring->length] = 0;
}

QString *qobject_to_qstring(const QObject *obj)
{
    if (qobject_type(obj) != QTYPE_QSTRING)
        return NULL;

    return container_of(obj, QString, base);
}

const char *qstring_get_str(const QString *qstring)
{
    return qstring->string;
}

static void qstring_destroy_obj(QObject *obj)
{
    QString *qs;

    assert(obj != NULL);
    qs = qobject_to_qstring(obj);
    qemu_free(qs->string);
    qemu_free(qs);
}
