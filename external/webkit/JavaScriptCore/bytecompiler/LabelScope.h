

#ifndef LabelScope_h
#define LabelScope_h

#include <wtf/PassRefPtr.h>
#include "Label.h"

namespace JSC {

    class Identifier;

    class LabelScope {
    public:
        enum Type { Loop, Switch, NamedLabel };

        LabelScope(Type type, const Identifier* name, int scopeDepth, PassRefPtr<Label> breakTarget, PassRefPtr<Label> continueTarget)
            : m_refCount(0)
            , m_type(type)
            , m_name(name)
            , m_scopeDepth(scopeDepth)
            , m_breakTarget(breakTarget)
            , m_continueTarget(continueTarget)
        {
        }

        void ref() { ++m_refCount; }
        void deref()
        {
            --m_refCount;
            ASSERT(m_refCount >= 0);
        }
        int refCount() const { return m_refCount; }

        Label* breakTarget() const { return m_breakTarget.get(); }
        Label* continueTarget() const { return m_continueTarget.get(); }

        Type type() const { return m_type; }
        const Identifier* name() const { return m_name; }
        int scopeDepth() const { return m_scopeDepth; }

    private:
        int m_refCount;
        Type m_type;
        const Identifier* m_name;
        int m_scopeDepth;
        RefPtr<Label> m_breakTarget;
        RefPtr<Label> m_continueTarget;
    };

} // namespace JSC

#endif // LabelScope_h
