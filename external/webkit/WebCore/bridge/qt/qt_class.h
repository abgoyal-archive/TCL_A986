

#ifndef BINDINGS_QT_CLASS_H_
#define BINDINGS_QT_CLASS_H_

#include "Bridge.h"
#include "qglobal.h"

QT_BEGIN_NAMESPACE
class QObject;
struct QMetaObject;
QT_END_NAMESPACE

namespace JSC {
namespace Bindings {


class QtClass : public Class {
protected:
    QtClass(const QMetaObject*);

public:
    static QtClass* classForObject(QObject*);
    virtual ~QtClass();

    virtual const char* name() const;
    virtual MethodList methodsNamed(const Identifier&, Instance*) const;
    virtual Field* fieldNamed(const Identifier&, Instance*) const;

    virtual JSValue fallbackObject(ExecState*, Instance*, const Identifier&);

private:
    QtClass(const QtClass&); // prohibit copying
    QtClass& operator=(const QtClass&); // prohibit assignment

    const QMetaObject* m_metaObject;
};

} // namespace Bindings
} // namespace JSC

#endif
