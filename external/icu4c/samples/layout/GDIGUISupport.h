
#ifndef __GDIGUISUPPORT_H
#define __GDIGUISUPPORT_H

#include "GUISupport.h"

class GDIGUISupport : public GUISupport
{
public:
    GDIGUISupport() {};
    virtual ~GDIGUISupport() {};

    virtual void postErrorMessage(const char *message, const char *title);
};

#endif
