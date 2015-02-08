

#ifndef qscriptvalue_p_h
#define qscriptvalue_p_h

#include "qscriptconverter_p.h"
#include "qscriptengine_p.h"
#include "qscriptvalue.h"
#include <JavaScriptCore/JavaScript.h>
#include <QtCore/qmath.h>
#include <QtCore/qnumeric.h>
#include <QtCore/qshareddata.h>
#include <QtCore/qvarlengtharray.h>

class QScriptEngine;
class QScriptValue;


class QScriptValuePrivate : public QSharedData {
public:
    inline static QScriptValuePrivate* get(const QScriptValue& q);
    inline static QScriptValue get(const QScriptValuePrivate* d);
    inline static QScriptValue get(QScriptValuePrivate* d);

    inline ~QScriptValuePrivate();

    inline QScriptValuePrivate();
    inline QScriptValuePrivate(const QString& string);
    inline QScriptValuePrivate(bool value);
    inline QScriptValuePrivate(int number);
    inline QScriptValuePrivate(uint number);
    inline QScriptValuePrivate(qsreal number);
    inline QScriptValuePrivate(QScriptValue::SpecialValue value);

    inline QScriptValuePrivate(const QScriptEngine* engine, bool value);
    inline QScriptValuePrivate(const QScriptEngine* engine, int value);
    inline QScriptValuePrivate(const QScriptEngine* engine, uint value);
    inline QScriptValuePrivate(const QScriptEngine* engine, qsreal value);
    inline QScriptValuePrivate(const QScriptEngine* engine, const QString& value);
    inline QScriptValuePrivate(const QScriptEngine* engine, QScriptValue::SpecialValue value);

    inline QScriptValuePrivate(const QScriptEnginePrivate* engine, JSValueRef value);
    inline QScriptValuePrivate(const QScriptEnginePrivate* engine, JSValueRef value, JSObjectRef object);

    inline bool isValid() const;
    inline bool isBool();
    inline bool isNumber();
    inline bool isNull();
    inline bool isString();
    inline bool isUndefined();
    inline bool isError();
    inline bool isObject();
    inline bool isFunction();

    inline QString toString() const;
    inline qsreal toNumber() const;
    inline bool toBool() const;
    inline qsreal toInteger() const;
    inline qint32 toInt32() const;
    inline quint32 toUInt32() const;
    inline quint16 toUInt16() const;

    inline bool equals(QScriptValuePrivate* other);
    inline bool strictlyEquals(const QScriptValuePrivate* other) const;
    inline bool assignEngine(QScriptEnginePrivate* engine);

    inline QScriptValuePrivate* call(const QScriptValuePrivate* , const QScriptValueList& args);

    inline JSGlobalContextRef context() const;
    inline JSValueRef value() const;
    inline JSObjectRef object() const;
    inline QScriptEnginePrivate* engine() const;

private:
    // Please, update class documentation when you change the enum.
    enum States {
        Invalid = 0,
        CString = 0x1000,
        CNumber,
        CBool,
        CSpecial,
        JSValue = 0x2000, // JS values are equal or higher then this value.
        JSNative,
        JSObject
    } m_state;
    QScriptEnginePtr m_engine;
    QString m_string;
    qsreal m_number;
    JSValueRef m_value;
    JSObjectRef m_object;

    inline void setValue(JSValueRef);

    inline bool inherits(const char*);

    inline bool isJSBased() const;
    inline bool isNumberBased() const;
    inline bool isStringBased() const;
};

QScriptValuePrivate* QScriptValuePrivate::get(const QScriptValue& q) { return q.d_ptr.data(); }

QScriptValue QScriptValuePrivate::get(const QScriptValuePrivate* d)
{
    return QScriptValue(const_cast<QScriptValuePrivate*>(d));
}

QScriptValue QScriptValuePrivate::get(QScriptValuePrivate* d)
{
    return QScriptValue(d);
}

QScriptValuePrivate::~QScriptValuePrivate()
{
    if (m_value)
        JSValueUnprotect(context(), m_value);
}

QScriptValuePrivate::QScriptValuePrivate()
    : m_state(Invalid)
    , m_value(0)
{
}

QScriptValuePrivate::QScriptValuePrivate(const QString& string)
    : m_state(CString)
    , m_string(string)
    , m_value(0)
{
}

QScriptValuePrivate::QScriptValuePrivate(bool value)
    : m_state(CBool)
    , m_number(value)
    , m_value(0)
{
}

QScriptValuePrivate::QScriptValuePrivate(int number)
    : m_state(CNumber)
    , m_number(number)
    , m_value(0)
{
}

QScriptValuePrivate::QScriptValuePrivate(uint number)
    : m_state(CNumber)
    , m_number(number)
    , m_value(0)
{
}

QScriptValuePrivate::QScriptValuePrivate(qsreal number)
    : m_state(CNumber)
    , m_number(number)
    , m_value(0)
{
}

QScriptValuePrivate::QScriptValuePrivate(QScriptValue::SpecialValue value)
    : m_state(CSpecial)
    , m_number(value)
    , m_value(0)
{
}

QScriptValuePrivate::QScriptValuePrivate(const QScriptEngine* engine, bool value)
    : m_state(JSNative)
{
    if (!engine) {
        // slower path reinitialization
        m_state = CBool;
        m_number = value;
        m_value = 0;
    } else {
        m_engine = QScriptEnginePrivate::get(engine);
        m_value = m_engine->makeJSValue(value);
        JSValueProtect(context(), m_value);
    }
}

QScriptValuePrivate::QScriptValuePrivate(const QScriptEngine* engine, int value)
    : m_state(JSNative)
{
    if (!engine) {
        // slower path reinitialization
        m_state = CNumber;
        m_number = value;
        m_value = 0;
    } else {
        m_engine = QScriptEnginePrivate::get(engine);
        m_value = m_engine->makeJSValue(value);
        JSValueProtect(context(), m_value);
    }
}

QScriptValuePrivate::QScriptValuePrivate(const QScriptEngine* engine, uint value)
    : m_state(JSNative)
{
    if (!engine) {
        // slower path reinitialization
        m_state = CNumber;
        m_number = value;
        m_value = 0;
    } else {
        m_engine = QScriptEnginePrivate::get(engine);
        m_value = m_engine->makeJSValue(value);
        JSValueProtect(context(), m_value);
    }
}

QScriptValuePrivate::QScriptValuePrivate(const QScriptEngine* engine, qsreal value)
    : m_state(JSNative)
{
    if (!engine) {
        // slower path reinitialization
        m_state = CNumber;
        m_number = value;
        m_value = 0;
    } else {
        m_engine = QScriptEnginePrivate::get(engine);
        m_value = m_engine->makeJSValue(value);
        JSValueProtect(context(), m_value);
    }
}

QScriptValuePrivate::QScriptValuePrivate(const QScriptEngine* engine, const QString& value)
    : m_state(JSNative)
{
    if (!engine) {
        // slower path reinitialization
        m_state = CString;
        m_string = value;
        m_value = 0;
    } else {
        m_engine = QScriptEnginePrivate::get(engine);
        m_value = m_engine->makeJSValue(value);
        JSValueProtect(context(), m_value);
    }
}

QScriptValuePrivate::QScriptValuePrivate(const QScriptEngine* engine, QScriptValue::SpecialValue value)
    : m_state(JSNative)
{
    if (!engine) {
        // slower path reinitialization
        m_state = CSpecial;
        m_number = value;
        m_value = 0;
    } else {
        m_engine = QScriptEnginePrivate::get(engine);
        m_value = m_engine->makeJSValue(value);
        JSValueProtect(context(), m_value);
    }
}

QScriptValuePrivate::QScriptValuePrivate(const QScriptEnginePrivate* engine, JSValueRef value)
    : m_state(JSValue)
    , m_engine(const_cast<QScriptEnginePrivate*>(engine))
    , m_value(value)
{
    Q_ASSERT(engine);
    JSValueProtect(context(), m_value);
}

QScriptValuePrivate::QScriptValuePrivate(const QScriptEnginePrivate* engine, JSValueRef value, JSObjectRef object)
    : m_state(JSObject)
    , m_engine(const_cast<QScriptEnginePrivate*>(engine))
    , m_value(value)
    , m_object(object)
{
    Q_ASSERT(engine);
    JSValueProtect(context(), m_value);
}

bool QScriptValuePrivate::isValid() const { return m_state != Invalid; }

bool QScriptValuePrivate::isBool()
{
    switch (m_state) {
    case CBool:
        return true;
    case JSValue:
        if (isObject())
            return false;
        // Fall-through.
    case JSNative:
        return JSValueIsBoolean(context(), value());
    default:
        return false;
    }
}

bool QScriptValuePrivate::isNumber()
{
    switch (m_state) {
    case CNumber:
        return true;
    case JSValue:
        if (isObject())
            return false;
        // Fall-through.
    case JSNative:
        return JSValueIsNumber(context(), value());
    default:
        return false;
    }
}

bool QScriptValuePrivate::isNull()
{
    switch (m_state) {
    case CSpecial:
        return m_number == static_cast<int>(QScriptValue::NullValue);
    case JSValue:
        if (isObject())
            return false;
        // Fall-through.
    case JSNative:
        return JSValueIsNull(context(), value());
    default:
        return false;
    }
}

bool QScriptValuePrivate::isString()
{
    switch (m_state) {
    case CString:
        return true;
    case JSValue:
        if (isObject())
            return false;
        // Fall-through.
    case JSNative:
        return JSValueIsString(context(), value());
    default:
        return false;
    }
}

bool QScriptValuePrivate::isUndefined()
{
    switch (m_state) {
    case CSpecial:
        return m_number == static_cast<int>(QScriptValue::UndefinedValue);
    case JSValue:
        if (isObject())
            return false;
        // Fall-through.
    case JSNative:
        return JSValueIsUndefined(context(), value());
    default:
        return false;
    }
}

bool QScriptValuePrivate::isError()
{
    switch (m_state) {
    case JSValue:
        if (!isObject())
            return false;
        // Fall-through.
    case JSObject:
        return inherits("Error");
    default:
        return false;
    }
}

bool QScriptValuePrivate::isObject()
{
    switch (m_state) {
    case JSObject:
        return true;
    case JSValue:
        m_object = JSValueToObject(context(), value(), /* exception */ 0);
        if (!m_object)
            return false;
        m_state = JSObject;
        return true;
    default:
        return false;
    }
}

bool QScriptValuePrivate::isFunction()
{
    switch (m_state) {
    case JSValue:
        m_object = JSValueToObject(context(), value(), /* exception */ 0);
        if (!m_object)
            return false;
        m_state = JSObject;
        // Fall-through.
    case JSObject:
        return JSObjectIsFunction(context(), object());
    default:
        return false;
    }
}

QString QScriptValuePrivate::toString() const
{
    switch (m_state) {
    case Invalid:
        return QString();
    case CBool:
        return m_number ? QString::fromLatin1("true") : QString::fromLatin1("false");
    case CString:
        return m_string;
    case CNumber:
        return QString::number(m_number);
    case CSpecial:
        return m_number == QScriptValue::NullValue ? QString::fromLatin1("null") : QString::fromLatin1("undefined");
    case JSValue:
    case JSNative:
    case JSObject:
        return QScriptConverter::toString(JSValueToStringCopy(context(), value(), /* exception */ 0));
    }

    Q_ASSERT_X(false, "toString()", "Not all states are included in the previous switch statement.");
    return QString(); // Avoid compiler warning.
}

qsreal QScriptValuePrivate::toNumber() const
{
    switch (m_state) {
    case JSValue:
    case JSNative:
    case JSObject:
        return JSValueToNumber(context(), value(), /* exception */ 0);
    case CNumber:
        return m_number;
    case CBool:
        return m_number ? 1 : 0;
    case Invalid:
        return 0;
    case CSpecial:
        return m_number == QScriptValue::NullValue ? 0 : qQNaN();
    case CString:
        bool ok;
        qsreal result = m_string.toDouble(&ok);
        if (ok)
            return result;
        result = m_string.toInt(&ok, 0); // Try other bases.
        if (ok)
            return result;
        if (m_string == "Infinity" || m_string == "-Infinity")
            return qInf();
        return m_string.length() ? qQNaN() : 0;
    }

    Q_ASSERT_X(false, "toNumber()", "Not all states are included in the previous switch statement.");
    return 0; // Avoid compiler warning.
}

bool QScriptValuePrivate::toBool() const
{
    switch (m_state) {
    case JSValue:
    case JSNative:
        return JSValueToBoolean(context(), value());
    case JSObject:
        return true;
    case CNumber:
        return !(qIsNaN(m_number) || !m_number);
    case CBool:
        return m_number;
    case Invalid:
    case CSpecial:
        return false;
    case CString:
        return m_string.length();
    }

    Q_ASSERT_X(false, "toBool()", "Not all states are included in the previous switch statement.");
    return false; // Avoid compiler warning.
}

qsreal QScriptValuePrivate::toInteger() const
{
    // TODO it is not true implementation!
    return toNumber();
}

qint32 QScriptValuePrivate::toInt32() const
{
    // TODO it is not true implementation!
    return toNumber();
}

quint32 QScriptValuePrivate::toUInt32() const
{
    // TODO it is not true implementation!
    return toNumber();
}

quint16 QScriptValuePrivate::toUInt16() const
{
    // TODO it is not true implementation!
    return toNumber();
}


bool QScriptValuePrivate::equals(QScriptValuePrivate* other)
{
    if (!isValid() || !other->isValid())
        return false;

    if ((m_state == other->m_state) && !isJSBased()) {
        if (isNumberBased())
            return m_number == other->m_number;
        return m_string == other->m_string;
    }

    if (isJSBased() && !other->isJSBased()) {
        if (!other->assignEngine(engine())) {
            qWarning("equals(): Cannot compare to a value created in a different engine");
            return false;
        }
    } else if (!isJSBased() && other->isJSBased()) {
        if (!other->assignEngine(other->engine())) {
            qWarning("equals(): Cannot compare to a value created in a different engine");
            return false;
        }
    }

    return JSValueIsEqual(context(), value(), other->value(), /* exception */ 0);
}

bool QScriptValuePrivate::strictlyEquals(const QScriptValuePrivate* other) const
{
    if (m_state != other->m_state)
        return false;
    if (isJSBased()) {
        if (other->engine() != engine()) {
            qWarning("strictlyEquals(): Cannot compare to a value created in a different engine");
            return false;
        }
        return JSValueIsStrictEqual(context(), value(), other->value());
    }
    if (isStringBased())
        return m_string == other->m_string;
    if (isNumberBased())
        return m_number == other->m_number;

    return false; // Invalid state.
}

bool QScriptValuePrivate::assignEngine(QScriptEnginePrivate* engine)
{
    JSValueRef value;
    switch (m_state) {
    case CBool:
        value = engine->makeJSValue(static_cast<bool>(m_number));
        break;
    case CString:
        value = engine->makeJSValue(m_string);
        break;
    case CNumber:
        value = engine->makeJSValue(m_number);
        break;
    case CSpecial:
        value = engine->makeJSValue(static_cast<QScriptValue::SpecialValue>(m_number));
        break;
    default:
        if (!isJSBased())
            Q_ASSERT_X(!isJSBased(), "assignEngine()", "Not all states are included in the previous switch statement.");
        else
            qWarning("JSValue can't be rassigned to an another engine.");
        return false;
    }
    m_engine = engine;
    m_state = JSNative;
    setValue(value);
    return true;
}

QScriptValuePrivate* QScriptValuePrivate::call(const QScriptValuePrivate*, const QScriptValueList& args)
{
    switch (m_state) {
    case JSValue:
        m_object = JSValueToObject(context(), value(), /* exception */ 0);
        if (!object()) {
            m_state = JSValue;
            return new QScriptValuePrivate;
        }
        m_state = JSObject;
        // Fall-through.
    case JSObject:
        {
            // Convert all arguments and bind to the engine.
            int argc = args.size();
            QVarLengthArray<JSValueRef, 8> argv(argc);
            QScriptValueList::const_iterator i = args.constBegin();
            for (int j = 0; i != args.constEnd(); j++, i++) {
                QScriptValuePrivate* value = QScriptValuePrivate::get(*i);
                if (!value->assignEngine(engine())) {
                    qWarning("QScriptValue::call() failed: cannot call function with values created in a different engine");
                    return new QScriptValuePrivate;
                }
                argv[j] = value->value();
            }

            // Make the call
            JSValueRef exception = 0;
            JSValueRef result = JSObjectCallAsFunction(context(), object(), /* thisObject */ 0, argc, argv.constData(), &exception);
            if (!result && exception)
                return new QScriptValuePrivate(engine(), exception);
            if (result && !exception)
                return new QScriptValuePrivate(engine(), result);
        }
        // this QSV is not a function <-- !result && !exception. Fall-through.
    default:
        return new QScriptValuePrivate;
    }
}

QScriptEnginePrivate* QScriptValuePrivate::engine() const
{
    // As long as m_engine is an autoinitializated pointer we can safely return it without
    // checking current state.
    return m_engine.data();
}

JSGlobalContextRef QScriptValuePrivate::context() const
{
    Q_ASSERT(isJSBased());
    return m_engine->context();
}

JSValueRef QScriptValuePrivate::value() const
{
    Q_ASSERT(isJSBased());
    return m_value;
}

JSObjectRef QScriptValuePrivate::object() const
{
    Q_ASSERT(m_state == JSObject);
    return m_object;
}

void QScriptValuePrivate::setValue(JSValueRef value)
{
    if (m_value)
        JSValueUnprotect(context(), m_value);
    if (value)
        JSValueProtect(context(), value);
    m_value = value;
}

bool QScriptValuePrivate::inherits(const char* name)
{
    Q_ASSERT(isJSBased());
    JSObjectRef globalObject = JSContextGetGlobalObject(context());
    JSValueRef error = JSObjectGetProperty(context(), globalObject, QScriptConverter::toString(name), 0);
    return JSValueIsInstanceOfConstructor(context(), value(), JSValueToObject(context(), error, /* exception */ 0), /* exception */ 0);
}

bool QScriptValuePrivate::isJSBased() const { return m_state >= JSValue; }

bool QScriptValuePrivate::isNumberBased() const { return !isJSBased() && !isStringBased() && m_state != Invalid; }

bool QScriptValuePrivate::isStringBased() const { return m_state == CString; }

#endif // qscriptvalue_p_h
