
#ifndef __GNOMEGUISUPPORT_H
#define __GNOMEGUISUPPORT_H

#include "GUISupport.h"

class GnomeGUISupport : public GUISupport
{
public:
    GnomeGUISupport() {};
    virtual ~GnomeGUISupport() {};

    virtual void postErrorMessage(const char *message, const char *title);
};

#endif
