

#ifndef ChildThreadDOMData_h
#define ChildThreadDOMData_h

#include "DOMData.h"
#include "ScopedDOMDataStore.h"

namespace WebCore {

    class ChildThreadDOMData : public DOMData {
    public:
        ChildThreadDOMData();
        DOMDataStore& getStore();

    private:
        ScopedDOMDataStore m_defaultStore;
    };

} // namespace WebCore

#endif // ChildThreadDOMData_h
