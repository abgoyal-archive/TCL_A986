

#include "config.h"
#include "PackageNotifier.h"

#if ENABLE(APPLICATION_INSTALLED)

#include <wtf/Assertions.h>
#include <wtf/StdLibExtras.h>

namespace WebCore {

PackageNotifier::PackageNotifier()
        : m_onResultAvailable(0), m_isInitialized(false), m_timer(this, &PackageNotifier::timerFired)  { }

void PackageNotifier::setOnResultAvailable(Callback callback)
{
    m_onResultAvailable = callback;
}

void PackageNotifier::addPackageNames(const HashSet<String>& packageNames)
{
    if (!m_isInitialized)
        m_isInitialized = true;

    typedef HashSet<String>::const_iterator NamesIterator;
    for (NamesIterator iter = packageNames.begin(); iter != packageNames.end(); ++iter)
        m_packageNames.add(*iter);

    if (m_onResultAvailable)
        m_onResultAvailable();
}

void PackageNotifier::addPackageName(const String& packageName)
{
    ASSERT(m_isInitialized);
    m_packageNames.add(packageName);
}

void PackageNotifier::removePackageName(const String& packageName)
{
    ASSERT(m_isInitialized);
    m_packageNames.remove(packageName);
}

void PackageNotifier::requestPackageResult()
{
    if (!m_isInitialized || m_timer.isActive())
        return;

    m_timer.startOneShot(0);
}

void PackageNotifier::timerFired(Timer<PackageNotifier>*)
{
    m_timer.stop();
    if (m_onResultAvailable)
        m_onResultAvailable();
}

bool PackageNotifier::isPackageInstalled(const String& packageName)
{
    return m_packageNames.contains(packageName);
}

PackageNotifier& packageNotifier()
{
    AtomicallyInitializedStatic(PackageNotifier*, packageNotifier = new PackageNotifier);
    return *packageNotifier;
}

}

#endif // ENABLE(APPLICATION_INSTALLED)
