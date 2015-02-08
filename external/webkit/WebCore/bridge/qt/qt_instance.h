

#ifndef BINDINGS_QT_INSTANCE_H_
#define BINDINGS_QT_INSTANCE_H_

#include "Bridge.h"
#include "runtime_root.h"
#include <QtScript/qscriptengine.h>
#include <qhash.h>
#include <qpointer.h>
#include <qset.h>

namespace JSC {

namespace Bindings {

class QtClass;
class QtField;
class QtRuntimeMetaMethod;

class QtInstance : public Instance {
public:
    ~QtInstance();

    virtual Class* getClass() const;
    virtual RuntimeObjectImp* newRuntimeObject(ExecState*);

    virtual void begin();
    virtual void end();

    virtual JSValue valueOf(ExecState*) const;
    virtual JSValue defaultValue(ExecState*, PreferredPrimitiveType) const;

    void markAggregate(MarkStack&);

    virtual JSValue invokeMethod(ExecState*, const MethodList&, const ArgList&);

    virtual void getPropertyNames(ExecState*, PropertyNameArray&);

    JSValue stringValue(ExecState* exec) const;
    JSValue numberValue(ExecState* exec) const;
    JSValue booleanValue() const;

    QObject* getObject() const { return m_object; }
    QObject* hashKey() const { return m_hashkey; }

    static PassRefPtr<QtInstance> getQtInstance(QObject*, PassRefPtr<RootObject>, QScriptEngine::ValueOwnership ownership);

    virtual bool getOwnPropertySlot(JSObject*, ExecState*, const Identifier&, PropertySlot&);
    virtual void put(JSObject*, ExecState*, const Identifier&, JSValue, PutPropertySlot&);

    void removeCachedMethod(JSObject*);

    static QtInstance* getInstance(JSObject*);

private:
    static PassRefPtr<QtInstance> create(QObject *instance, PassRefPtr<RootObject> rootObject, QScriptEngine::ValueOwnership ownership)
    {
        return adoptRef(new QtInstance(instance, rootObject, ownership));
    }

    friend class QtClass;
    friend class QtField;
    QtInstance(QObject*, PassRefPtr<RootObject>, QScriptEngine::ValueOwnership ownership); // Factory produced only..
    mutable QtClass* m_class;
    QPointer<QObject> m_object;
    QObject* m_hashkey;
    mutable QHash<QByteArray, JSObject*> m_methods;
    mutable QHash<QString, QtField*> m_fields;
    mutable QtRuntimeMetaMethod* m_defaultMethod;
    QScriptEngine::ValueOwnership m_ownership;
};

} // namespace Bindings

} // namespace JSC

#endif
