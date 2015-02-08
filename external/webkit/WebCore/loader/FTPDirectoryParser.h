


// This was originally Mozilla code, titled ParseFTPList.h
// Original version of this file can currently be found at: http://mxr.mozilla.org/mozilla1.8/source/netwerk/streamconv/converters/ParseFTPList.h

#ifndef FTPDirectoryParser_h
#define FTPDirectoryParser_h

#include "PlatformString.h"

#include <time.h>

#define SUPPORT_LSL  /* Support for /bin/ls -l and dozens of variations therof */
#define SUPPORT_DLS  /* Support for /bin/dls format (very, Very, VERY rare) */
#define SUPPORT_EPLF /* Support for Extraordinarily Pathetic List Format */
#define SUPPORT_DOS  /* Support for WinNT server in 'site dirstyle' dos */
#define SUPPORT_VMS  /* Support for VMS (all: MultiNet, UCX, CMU-IP) */
#define SUPPORT_CMS  /* Support for IBM VM/CMS,VM/ESA (z/VM and LISTING forms) */
#define SUPPORT_OS2  /* Support for IBM TCP/IP for OS/2 - FTP Server */
#define SUPPORT_W16  /* Support for win16 hosts: SuperTCP or NetManage Chameleon */

namespace WebCore {

typedef struct tm FTPTime;

struct ListState {    
    ListState()
        : now(0)
        , listStyle(0)
        , parsedOne(false)
        , carryBufferLength(0)
        , numLines(0)
    { 
        memset(&nowFTPTime, 0, sizeof(FTPTime));
    }
    
    double      now;               /* needed for year determination */
    FTPTime     nowFTPTime;
    char        listStyle;         /* LISTing style */
    bool        parsedOne;         /* returned anything yet? */
    char        carryBuffer[84];   /* for VMS multiline */
    int         carryBufferLength; /* length of name in carry_buf */
    int64_t     numLines;          /* number of lines seen */
};

enum FTPEntryType {
    FTPDirectoryEntry,
    FTPFileEntry,
    FTPLinkEntry,
    FTPMiscEntry,
    FTPJunkEntry
};

struct ListResult
{
    ListResult()
    { 
        clear();
    }
    
    void clear()
    {
        valid = false;
        type = FTPJunkEntry;
        filename = 0;
        filenameLength = 0;
        linkname = 0;
        linknameLength = 0;
        fileSize.truncate(0);
        caseSensitive = false;
        memset(&modifiedTime, 0, sizeof(FTPTime));
    }
    
    bool valid;
    FTPEntryType type;        
    
    const char* filename;
    uint32_t filenameLength;
    
    const char* linkname;
    uint32_t linknameLength;
    
    String fileSize;      
    FTPTime modifiedTime; 
    bool caseSensitive; // file system is definitely case insensitive
};

FTPEntryType parseOneFTPLine(const char* inputLine, ListState&, ListResult&);
                 
} // namespace WebCore

#endif // FTPDirectoryParser_h
