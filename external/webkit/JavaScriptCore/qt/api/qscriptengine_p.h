

#ifndef qscriptengine_p_h
#define qscriptengine_p_h

#include "qscriptconverter_p.h"
#include "qscriptengine.h"
#include "qscriptvalue.h"
#include <JavaScriptCore/JavaScript.h>
#include <QtCore/qshareddata.h>
#include <QtCore/qstring.h>

class QScriptEngine;

class QScriptEnginePrivate : public QSharedData {
public:
    static QScriptEnginePtr get(const QScriptEngine* q) { Q_ASSERT(q); return q->d_ptr; }
    static QScriptEngine* get(const QScriptEnginePrivate* d) { Q_ASSERT(d); return d->q_ptr; }

    QScriptEnginePrivate(const QScriptEngine*);
    ~QScriptEnginePrivate();

    QScriptValuePrivate* evaluate(const QString& program, const QString& fileName, int lineNumber);
    inline void collectGarbage();

    inline JSValueRef makeJSValue(double number) const;
    inline JSValueRef makeJSValue(int number) const;
    inline JSValueRef makeJSValue(uint number) const;
    inline JSValueRef makeJSValue(const QString& string) const;
    inline JSValueRef makeJSValue(bool number) const;
    inline JSValueRef makeJSValue(QScriptValue::SpecialValue value) const;

    inline JSGlobalContextRef context() const;
private:
    QScriptEngine* q_ptr;
    JSGlobalContextRef m_context;
};

void QScriptEnginePrivate::collectGarbage()
{
    JSGarbageCollect(m_context);
}

JSValueRef QScriptEnginePrivate::makeJSValue(double number) const
{
    return JSValueMakeNumber(m_context, number);
}

JSValueRef QScriptEnginePrivate::makeJSValue(int number) const
{
    return JSValueMakeNumber(m_context, number);
}

JSValueRef QScriptEnginePrivate::makeJSValue(uint number) const
{
    return JSValueMakeNumber(m_context, number);
}

JSValueRef QScriptEnginePrivate::makeJSValue(const QString& string) const
{
    return JSValueMakeString(m_context, QScriptConverter::toString(string));
}

JSValueRef QScriptEnginePrivate::makeJSValue(bool value) const
{
    return JSValueMakeBoolean(m_context, value);
}

JSValueRef QScriptEnginePrivate::makeJSValue(QScriptValue::SpecialValue value) const
{
    if (value == QScriptValue::NullValue)
        return JSValueMakeNull(m_context);
    return JSValueMakeUndefined(m_context);
}

JSGlobalContextRef QScriptEnginePrivate::context() const
{
    return m_context;
}

#endif
