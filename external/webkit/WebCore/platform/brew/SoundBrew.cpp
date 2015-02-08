

#include "config.h"
#include "Sound.h"

#include <AEEAppGen.h>
#include <AEEStdLib.h>

namespace WebCore {

void systemBeep()
{
    // systemBeep is used by Editor to notify errors when a user cuts or pastes
    // where these operations are not permitted.
    // Use BEEP_ERROR with loud volume.
    IShell* shell = reinterpret_cast<AEEApplet*>(GETAPPINSTANCE())->m_pIShell;
    ISHELL_Beep(shell, BEEP_ERROR, TRUE);
}

} // namespace WebCore

