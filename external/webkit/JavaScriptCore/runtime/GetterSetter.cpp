

#include "config.h"
#include "GetterSetter.h"

#include "JSObject.h"
#include <wtf/Assertions.h>

namespace JSC {

void GetterSetter::markChildren(MarkStack& markStack)
{
    JSCell::markChildren(markStack);

    if (m_getter)
        markStack.append(m_getter);
    if (m_setter)
        markStack.append(m_setter);
}

bool GetterSetter::isGetterSetter() const
{
    return true;
}

} // namespace JSC
