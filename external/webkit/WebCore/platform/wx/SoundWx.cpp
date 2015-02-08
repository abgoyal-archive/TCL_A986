

#include "config.h"

#include <wx/defs.h>
#include <wx/utils.h>

#include "Sound.h"

namespace WebCore {

void systemBeep()
{
    wxBell();
}

}

// vim: ts=4 sw=4 et
