

#ifndef AccessibilityObjectWrapper_h
#define AccessibilityObjectWrapper_h

#include <wtf/RefCounted.h>

namespace WebCore {

    class AccessibilityObject;
    class AccessibilityObjectWrapper : public RefCounted<AccessibilityObjectWrapper> {
    public:
        virtual ~AccessibilityObjectWrapper() {}
        virtual void detach() = 0;
        bool attached() const { return m_object; }
        AccessibilityObject* accessibilityObject() const { return m_object; }

    protected:
        AccessibilityObjectWrapper(AccessibilityObject* obj)
            : m_object(obj)
        {
        }
        AccessibilityObjectWrapper() : m_object(0) { }

        AccessibilityObject* m_object;
    };

} // namespace WebCore

#endif
