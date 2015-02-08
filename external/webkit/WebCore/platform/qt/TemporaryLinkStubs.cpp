

#include "config.h"

#include "AXObjectCache.h"
#include "DNS.h"
#include "CString.h"
#include "CachedResource.h"
#include "CookieJar.h"
#include "Cursor.h"
#include "Font.h"
#include "Frame.h"
#include "FrameLoader.h"
#include "FTPDirectoryDocument.h"
#include "IntPoint.h"
#include "Widget.h"
#include "GraphicsContext.h"
#include "Cursor.h"
#include "loader.h"
#include "FileSystem.h"
#include "FrameView.h"
#include "GraphicsContext.h"
#include "IconLoader.h"
#include "IntPoint.h"
#include "KURL.h"
#include "Language.h"
#include "loader.h"
#include "LocalizedStrings.h"
#include "Node.h"
#include "NotImplemented.h"
#include "Path.h"
#include "PlatformMouseEvent.h"
#include "PluginDatabase.h"
#include "PluginPackage.h"
#include "PluginView.h"
#include "RenderTheme.h"
#include "SharedBuffer.h"
#include "SystemTime.h"
#include "TextBoundaries.h"
#include "Widget.h"
#include <stdio.h>
#include <stdlib.h>
#include <float.h>

using namespace WebCore;

#if defined(Q_OS_WINCE)
Vector<String> PluginDatabase::defaultPluginDirectories()
{
    notImplemented();
    return Vector<String>();
}

void PluginDatabase::getPluginPathsInDirectories(HashSet<String>& paths) const
{
    notImplemented();
}

bool PluginDatabase::isPreferredPluginDirectory(const String& directory)
{
    notImplemented();
    return false;
}
#endif

namespace WebCore {

void getSupportedKeySizes(Vector<String>&)
{
    notImplemented();
}

String signedPublicKeyAndChallengeString(unsigned, const String&, const KURL&)
{
    return String();
}

#if !defined(Q_OS_WIN)
// defined in win/SystemTimeWin.cpp, which is compiled for the Qt/Windows port
float userIdleTime()
{
    notImplemented();
    return FLT_MAX; // return an arbitrarily high userIdleTime so that releasing pages from the page cache isn't postponed
}
#endif

}

// vim: ts=4 sw=4 et
