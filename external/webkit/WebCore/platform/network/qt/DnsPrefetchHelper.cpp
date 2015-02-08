

#include "config.h"
#include "DnsPrefetchHelper.h"

#include "CString.h"

namespace WebCore {
// this is called on mouse over a href and on page loading
void prefetchDNS(const String& hostname)
{
    if (QWebSettings::globalSettings()->testAttribute(QWebSettings::DnsPrefetchEnabled)) {
        static DnsPrefetchHelper dnsPrefetchHelper;
        dnsPrefetchHelper.lookup(QString(hostname));
    }
}

}
