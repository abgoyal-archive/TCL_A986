

#ifndef MainThreadDOMData_h
#define MainThreadDOMData_h

#include "DOMData.h"
#include "StaticDOMDataStore.h"

namespace WebCore {

    class MainThreadDOMData : public DOMData {
    public:
        static MainThreadDOMData* getCurrent(); // Caller must be on the main thread.
        static DOMDataStore& getCurrentMainThreadStore();
                
        virtual DOMDataStore& getStore() { return getMainThreadStore(); }
        

    private:
        MainThreadDOMData();
        DOMDataStore& getMainThreadStore();

        StaticDOMDataStore m_defaultStore;
        // Note: The DOMDataStores for isolated world are owned by the world object.
    };

} // namespace WebCore

#endif // MainThreadDOMData_h
