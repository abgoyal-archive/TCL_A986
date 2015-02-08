

#include "config.h"
#include "Logging.h"

#include "CString.h"
#include "PlatformString.h"
#include <wtf/Vector.h>

#include <wx/defs.h>
#include <wx/utils.h>

namespace WebCore {

void InitializeLoggingChannelsIfNecessary()
{
    static bool haveInitializedLoggingChannels = false;
    if (haveInitializedLoggingChannels)
        return;

    haveInitializedLoggingChannels = true;

    wxString loggingEnv;
    wxGetEnv(wxT("WX_WEBKIT_LOG"), &loggingEnv);
    if (loggingEnv == wxEmptyString)
        return;

    String wkLoggingEnv = loggingEnv;
    Vector<String> logs;
    
    wkLoggingEnv.split(",", logs);
    
    for (size_t i = 0; i < logs.size(); ++i) {
        WTFLogChannel* channel = getChannelFromName(logs[i]);
        
        if (channel)
            channel->state = WTFLogChannelOn;
    }
}

}
