

#include "config.h"

#include "qscriptengine.h"

#include "qscriptengine_p.h"
#include "qscriptvalue_p.h"

QScriptEngine::QScriptEngine()
    : d_ptr(new QScriptEnginePrivate(this))
{
}

QScriptEngine::~QScriptEngine()
{
}

QScriptValue QScriptEngine::evaluate(const QString& program, const QString& fileName, int lineNumber)
{
    return QScriptValuePrivate::get(d_ptr->evaluate(program, fileName, lineNumber));
}

void QScriptEngine::collectGarbage()
{
    d_ptr->collectGarbage();
}

QScriptValue QScriptEngine::nullValue()
{
    return QScriptValue(this, QScriptValue::NullValue);
}

QScriptValue QScriptEngine::undefinedValue()
{
    return QScriptValue(this, QScriptValue::UndefinedValue);
}
