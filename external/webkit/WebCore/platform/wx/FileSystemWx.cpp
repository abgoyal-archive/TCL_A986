

#include "config.h"
#include "FileSystem.h"

#include "CString.h"
#include "NotImplemented.h"
#include "PlatformString.h"

#include <wx/wx.h>
#include <wx/datetime.h>
#include <wx/dir.h>
#include <wx/dynlib.h>
#include <wx/file.h>
#include <wx/filefn.h>
#include <wx/filename.h>

#if OS(DARWIN)
#include <CoreFoundation/CoreFoundation.h>
#endif

namespace WebCore {

bool fileExists(const String& path)
{
    // NOTE: This is called for directory paths too so we need to check both.
    return wxFileName::FileExists(path) || wxFileName::DirExists(path);
}

bool deleteFile(const String& path)
{
    return wxRemoveFile(path);
}

bool deleteEmptyDirectory(const String& path)
{
    return wxFileName::Rmdir(path);
}

bool getFileSize(const String& path, long long& resultSize)
{
    wxULongLong size = wxFileName::GetSize(path);
    if (wxInvalidSize != size) {
        // TODO: why is FileSystem::getFileSize signed?
        resultSize = (long long)size.GetValue();
        return true;
    }

    return false;
}

bool getFileModificationTime(const String& path, time_t& t)
{
    t = wxFileName(path).GetModificationTime().GetTicks();
    return true;
}

bool makeAllDirectories(const String& path)
{
    return wxFileName::Mkdir(path, 0777, wxPATH_MKDIR_FULL);
}

String pathByAppendingComponent(const String& path, const String& component)
{
    return wxFileName(path, component).GetFullPath();
}

String homeDirectoryPath()
{
    return wxFileName::GetHomeDir();
}

String pathGetFileName(const String& path)
{
    return wxFileName(path).GetFullName();
}

String directoryName(const String& path)
{
    return wxFileName(path).GetPath();
}

CString openTemporaryFile(const char* prefix, PlatformFileHandle& handle)
{
    notImplemented();
    handle = invalidPlatformFileHandle;
    return CString();
}

void closeFile(PlatformFileHandle&)
{
    notImplemented();
}

int writeToFile(PlatformFileHandle, const char* data, int length)
{
    notImplemented();
    return 0;
}

bool unloadModule(PlatformModule mod)
{
#if OS(WINDOWS)
    return ::FreeLibrary(mod);
#elif OS(DARWIN)
    CFRelease(mod);
    return true;
#else
    wxASSERT(mod);
    delete mod;
    return true;
#endif
}


class wxDirTraverserNonRecursive : public wxDirTraverser {
public:
    wxDirTraverserNonRecursive(wxString basePath, wxArrayString& files) : m_basePath(basePath), m_files(files) { }

    virtual wxDirTraverseResult OnFile(const wxString& filename)
    {
        wxFileName afile(filename);
        afile.MakeRelativeTo(m_basePath);
        if (afile.GetFullPath().Find(afile.GetPathSeparator()) == wxNOT_FOUND)
            m_files.push_back(filename);

        return wxDIR_CONTINUE;
    }

    virtual wxDirTraverseResult OnDir(const wxString& dirname)
    {
        wxFileName dirfile(dirname);
        dirfile.MakeRelativeTo(m_basePath);
        if (dirfile.GetFullPath().Find(dirfile.GetPathSeparator()) == wxNOT_FOUND)
            m_files.push_back(dirname);

        return wxDIR_CONTINUE;
    }

private:
    wxString m_basePath;
    wxArrayString& m_files;

    DECLARE_NO_COPY_CLASS(wxDirTraverserNonRecursive)
};

Vector<String> listDirectory(const String& path, const String& filter)
{
    wxArrayString   file_paths;
    // wxDir::GetAllFiles recurses and for platforms like Mac where
    // a .plugin or .bundle can be a dir wx will recurse into the bundle
    // and list the files rather than just returning the plugin name, so
    // we write a special traverser that works around that issue.
    wxDirTraverserNonRecursive traverser(path, file_paths);
    
    wxDir dir(path);
    dir.Traverse(traverser, _T(""), wxDIR_FILES | wxDIR_DIRS);

    Vector<String> entries;
    
    for (int i = 0; i < file_paths.GetCount(); i++)
    {
        entries.append(file_paths[i]);
    }   
    
    return entries;
}

}
