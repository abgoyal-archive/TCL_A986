

#include "config.h"

#include "qscriptvalue.h"

#include "qscriptengine.h"
#include "qscriptengine_p.h"
#include "qscriptvalue_p.h"
#include <QtCore/qdebug.h>

QScriptValue::QScriptValue()
    : d_ptr(new QScriptValuePrivate())
{
}

QScriptValue::QScriptValue(bool value)
    : d_ptr(new QScriptValuePrivate(value))
{
}

QScriptValue::QScriptValue(int value)
    : d_ptr(new QScriptValuePrivate(value))
{
}

QScriptValue::QScriptValue(uint value)
    : d_ptr(new QScriptValuePrivate(value))
{
}

QScriptValue::QScriptValue(qsreal value)
    : d_ptr(new QScriptValuePrivate(value))
{
}

QScriptValue::QScriptValue(const QString& value)
    : d_ptr(new QScriptValuePrivate(value))
{
}

QScriptValue::QScriptValue(SpecialValue value)
    : d_ptr(new QScriptValuePrivate(value))
{
}

QScriptValue::QScriptValue(const char* value)
    : d_ptr(new QScriptValuePrivate(QString::fromUtf8(value)))
{
}

QScriptValue::QScriptValue(void* d)
{
    Q_ASSERT(false);
}

QScriptValue::QScriptValue(QScriptValuePrivate* d)
    : d_ptr(d)
{
}

QScriptValue::QScriptValue(QScriptEngine* engine, bool value)
    : d_ptr(new QScriptValuePrivate(engine, value))
{
}

QScriptValue::QScriptValue(QScriptEngine* engine, int value)
    : d_ptr(new QScriptValuePrivate(engine, value))
{
}

QScriptValue::QScriptValue(QScriptEngine* engine, uint value)
    : d_ptr(new QScriptValuePrivate(engine, value))
{
}

QScriptValue::QScriptValue(QScriptEngine* engine, qsreal value)
    : d_ptr(new QScriptValuePrivate(engine, value))
{
}

QScriptValue::QScriptValue(QScriptEngine* engine, const QString& value)
    : d_ptr(new QScriptValuePrivate(engine, value))
{
}

QScriptValue::QScriptValue(QScriptEngine* engine, const char* value)
    : d_ptr(new QScriptValuePrivate(engine, QString::fromUtf8(value)))
{
}

QScriptValue::QScriptValue(QScriptEngine* engine, SpecialValue value)
    : d_ptr(new QScriptValuePrivate(engine, value))
{
}

QScriptValue::QScriptValue(const QScriptValue& other)
    : d_ptr(other.d_ptr)
{
}

QScriptValue::~QScriptValue()
{
}

bool QScriptValue::isValid() const
{
    return d_ptr->isValid();
}

bool QScriptValue::isBool() const
{
    return d_ptr->isBool();
}

bool QScriptValue::isBoolean() const
{
    return d_ptr->isBool();
}

bool QScriptValue::isNumber() const
{
    return d_ptr->isNumber();
}

bool QScriptValue::isNull() const
{
    return d_ptr->isNull();
}

bool QScriptValue::isString() const
{
    return d_ptr->isString();
}

bool QScriptValue::isUndefined() const
{
    return d_ptr->isUndefined();
}

bool QScriptValue::isError() const
{
    return d_ptr->isError();
}

bool QScriptValue::isObject() const
{
    return d_ptr->isObject();
}

bool QScriptValue::isFunction() const
{
    return d_ptr->isFunction();
}

QString QScriptValue::toString() const
{
    return d_ptr->toString();
}

qsreal QScriptValue::toNumber() const
{
    return d_ptr->toNumber();
}

bool QScriptValue::toBool() const
{
    return d_ptr->toBool();
}

bool QScriptValue::toBoolean() const
{
    return d_ptr->toBool();
}

qsreal QScriptValue::toInteger() const
{
    return d_ptr->toInteger();
}

qint32 QScriptValue::toInt32() const
{
    return d_ptr->toInt32();
}

quint32 QScriptValue::toUInt32() const
{
    return d_ptr->toUInt32();
}

quint16 QScriptValue::toUInt16() const
{
    return d_ptr->toUInt16();
}

QScriptValue QScriptValue::call(const QScriptValue& thisObject, const QScriptValueList& args)
{
    return d_ptr->call(thisObject.d_ptr.data(), args);
}

QScriptEngine* QScriptValue::engine() const
{
    QScriptEnginePrivate* engine = d_ptr->engine();
    if (engine)
        return QScriptEnginePrivate::get(engine);
    return 0;
}

QScriptValue& QScriptValue::operator=(const QScriptValue& other)
{
    d_ptr = other.d_ptr;
    return *this;
}

bool QScriptValue::equals(const QScriptValue& other) const
{
    return d_ptr == other.d_ptr || d_ptr->equals(QScriptValuePrivate::get(other));
}

bool QScriptValue::strictlyEquals(const QScriptValue& other) const
{
    return d_ptr == other.d_ptr || d_ptr->strictlyEquals(QScriptValuePrivate::get(other));
}
