
#include "config.h"
#include "PluginDatabase.h"

#include <QFileInfo>
#include <f32file.h>

static const char QTPLUGIN_FILTER[] = "*.qtplugin";
static const char QT_PLUGIN_FOLDER[] = ":\\resource\\qt\\plugins\\npqtplugins\\";

namespace WebCore {

Vector<String> PluginDatabase::defaultPluginDirectories()
{
    Vector<String> directories;
    //find the installation drive
    TDriveList drivelist;
    TChar driveLetter;
    RFs fsSession;
    
    if (fsSession.Connect() == KErrNone && fsSession.DriveList(drivelist) == KErrNone) {
        for (TInt driveNumber = EDriveA; driveNumber <= EDriveZ; driveNumber++) {
            if (drivelist[driveNumber] && fsSession.DriveToChar(driveNumber, driveLetter) == KErrNone) {
                QString driveStringValue(QChar((uint)driveLetter.GetUpperCase()));
                QString stubDirPath;
                stubDirPath.append(driveStringValue);
                stubDirPath.append(QT_PLUGIN_FOLDER);
                if (QFileInfo(stubDirPath).exists())
                    directories.append(stubDirPath);
            }
        }
    }

    fsSession.Close();
    return directories;
}

bool PluginDatabase::isPreferredPluginDirectory(const String& path)
{
    return true;
}

void PluginDatabase::getPluginPathsInDirectories(HashSet<String>& paths) const
{
    // FIXME: This should be a case insensitive set.
    HashSet<String> uniqueFilenames;

    String fileNameFilter(QTPLUGIN_FILTER);

    Vector<String>::const_iterator dirsEnd = m_pluginDirectories.end();
    for (Vector<String>::const_iterator dIt = m_pluginDirectories.begin(); dIt != dirsEnd; ++dIt) {
        Vector<String> pluginPaths = listDirectory(*dIt, fileNameFilter);
        Vector<String>::const_iterator pluginsEnd = pluginPaths.end();
        for (Vector<String>::const_iterator pIt = pluginPaths.begin(); pIt != pluginsEnd; ++pIt) {
            if (!fileExists(*pIt))
                continue;
            paths.add(*pIt);
        }
    }
}

}
