

#ifndef WorkerNavigator_h
#define WorkerNavigator_h

#if ENABLE(WORKERS)

#include "NavigatorBase.h"
#include "PlatformString.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/RefPtr.h>

namespace WebCore {

    class WorkerNavigator : public NavigatorBase, public RefCounted<WorkerNavigator> {
    public:
        static PassRefPtr<WorkerNavigator> create(const String& userAgent) { return adoptRef(new WorkerNavigator(userAgent)); }
        virtual ~WorkerNavigator();

        virtual String userAgent() const;

    private:
        WorkerNavigator(const String&);

        String m_userAgent;
    };

} // namespace WebCore

#endif // ENABLE(WORKERS)

#endif // WorkerNavigator_h
