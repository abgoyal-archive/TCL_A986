

#include "config.h"
#include "JSVariableObject.h"

#include "PropertyNameArray.h"
#include "PropertyDescriptor.h"

namespace JSC {

bool JSVariableObject::deleteProperty(ExecState* exec, const Identifier& propertyName)
{
    if (symbolTable().contains(propertyName.ustring().rep()))
        return false;

    return JSObject::deleteProperty(exec, propertyName);
}

void JSVariableObject::getOwnPropertyNames(ExecState* exec, PropertyNameArray& propertyNames, EnumerationMode mode)
{
    SymbolTable::const_iterator end = symbolTable().end();
    for (SymbolTable::const_iterator it = symbolTable().begin(); it != end; ++it) {
        if (!(it->second.getAttributes() & DontEnum) || (mode == IncludeDontEnumProperties))
            propertyNames.add(Identifier(exec, it->first.get()));
    }
    
    JSObject::getOwnPropertyNames(exec, propertyNames, mode);
}

bool JSVariableObject::isVariableObject() const
{
    return true;
}

bool JSVariableObject::symbolTableGet(const Identifier& propertyName, PropertyDescriptor& descriptor)
{
    SymbolTableEntry entry = symbolTable().inlineGet(propertyName.ustring().rep());
    if (!entry.isNull()) {
        descriptor.setDescriptor(registerAt(entry.getIndex()).jsValue(), entry.getAttributes() | DontDelete);
        return true;
    }
    return false;
}

} // namespace JSC
