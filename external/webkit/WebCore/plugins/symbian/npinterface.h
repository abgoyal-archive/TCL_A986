
#ifndef npinterface_H
#define npinterface_H

#include "npfunctions.h"
#include <QtPlugin>

class NPInterface {
public:
    virtual NPError NP_Initialize(NPNetscapeFuncs* aNPNFuncs, NPPluginFuncs* aNPPFuncs) = 0;
    virtual void NP_Shutdown() = 0;
    virtual char* NP_GetMIMEDescription() = 0;
};


QT_BEGIN_NAMESPACE
Q_DECLARE_INTERFACE(NPInterface, "com.nokia.qts60.webplugin/1.0");
QT_END_NAMESPACE

#endif // npinterface_H
