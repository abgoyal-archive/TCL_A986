
#include <windows.h>

#include "GDIGUISupport.h"

void GDIGUISupport::postErrorMessage(const char *message, const char *title)
{
    MessageBoxA(NULL, message, title, MB_ICONERROR);
}

