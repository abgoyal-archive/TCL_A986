

#ifndef BINDINGS_C_RUNTIME_H_
#define BINDINGS_C_RUNTIME_H_

#if ENABLE(NETSCAPE_PLUGIN_API)

#include "Bridge.h"
#include "npruntime_internal.h"

namespace JSC {
namespace Bindings {

class CField : public Field {
public:
    CField(NPIdentifier ident) : _fieldIdentifier(ident) { }

    virtual JSValue valueFromInstance(ExecState*, const Instance*) const;
    virtual void setValueToInstance(ExecState*, const Instance*, JSValue) const;

    NPIdentifier identifier() const { return _fieldIdentifier; }

private:
    NPIdentifier _fieldIdentifier;
};


class CMethod : public Method
{
public:
    CMethod(NPIdentifier ident) : _methodIdentifier(ident) { }

    NPIdentifier identifier() const { return _methodIdentifier; }
    virtual int numParameters() const { return 0; }

private:
    NPIdentifier _methodIdentifier;
};

} // namespace Bindings
} // namespace JSC

#endif // ENABLE(NETSCAPE_PLUGIN_API)

#endif
