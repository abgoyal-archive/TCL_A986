

#ifndef PackageNotifier_h
#define PackageNotifier_h

#if ENABLE(APPLICATION_INSTALLED)

#include "PlatformString.h"
#include "StringHash.h"
#include "Timer.h"
#include <wtf/HashSet.h>
#include <wtf/Noncopyable.h>

namespace WebCore {

class PackageNotifier : public Noncopyable {
public:
    PackageNotifier();

    typedef void (*Callback)();
    void setOnResultAvailable(Callback);

    void addPackageNames(const HashSet<String>& packageNames);
    void addPackageName(const String& packageName);
    void removePackageName(const String& packageName);

    void requestPackageResult();
    bool isPackageInstalled(const String& packageName);

private:
    void timerFired(Timer<PackageNotifier>*);

    Callback m_onResultAvailable;

    HashSet<String> m_packageNames;
    bool m_isInitialized;
    Timer<PackageNotifier> m_timer;
};

PackageNotifier& packageNotifier();

}

#endif

#endif // PackageNotifier_h