
#ifndef __GUISUPPORT_H
#define __GUISUPPORT_H

class GUISupport
{
public:
    GUISupport() {};
    virtual ~GUISupport() {};

    virtual void postErrorMessage(const char *message, const char *title) = 0;
};

#endif
