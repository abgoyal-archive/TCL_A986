

#include "config.h"
#include <qwebkitversion.h>
#include <WebKitVersion.h>

QString qWebKitVersion()
{
    return QString("%1.%2").arg(WEBKIT_MAJOR_VERSION).arg(WEBKIT_MINOR_VERSION);
}

int qWebKitMajorVersion()
{
    return WEBKIT_MAJOR_VERSION;
}

int qWebKitMinorVersion()
{
    return WEBKIT_MINOR_VERSION;
}
