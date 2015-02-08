

#ifndef qscriptengine_h
#define qscriptengine_h

#include <QtCore/qobject.h>
#include <QtCore/qshareddata.h>
#include <QtCore/qstring.h>

class QScriptValue;
class QScriptEnginePrivate;

// Internal typedef
typedef QExplicitlySharedDataPointer<QScriptEnginePrivate> QScriptEnginePtr;

class QScriptEngine : public QObject {
public:
    QScriptEngine();
    ~QScriptEngine();

    QScriptValue evaluate(const QString& program, const QString& fileName = QString(), int lineNumber = 1);
    void collectGarbage();

    QScriptValue nullValue();
    QScriptValue undefinedValue();
private:
    friend class QScriptEnginePrivate;

    QScriptEnginePtr d_ptr;
};

#endif
