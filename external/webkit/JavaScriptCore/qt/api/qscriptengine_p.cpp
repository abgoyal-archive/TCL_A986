

#include "config.h"

#include "qscriptengine_p.h"

#include "qscriptvalue_p.h"

QScriptEnginePrivate::QScriptEnginePrivate(const QScriptEngine* engine)
    : q_ptr(const_cast<QScriptEngine*>(engine))
    , m_context(JSGlobalContextCreate(0))
{
}

QScriptEnginePrivate::~QScriptEnginePrivate()
{
    JSGlobalContextRelease(m_context);
}

QScriptValuePrivate* QScriptEnginePrivate::evaluate(const QString& program, const QString& fileName, int lineNumber)
{
    JSStringRef script = QScriptConverter::toString(program);
    JSStringRef file = QScriptConverter::toString(fileName);
    JSValueRef exception;
    JSValueRef result = JSEvaluateScript(m_context, script, /* Global Object */ 0, file, lineNumber, &exception);
    if (!result)
        return new QScriptValuePrivate(this, exception); // returns an exception
    return new QScriptValuePrivate(this, result);
}
