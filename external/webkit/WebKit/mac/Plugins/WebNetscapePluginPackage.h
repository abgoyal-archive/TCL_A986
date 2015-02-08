

#if ENABLE(NETSCAPE_PLUGIN_API)
#import "WebBasePluginPackage.h"

#ifdef BUILDING_ON_TIGER
typedef short ResFileRefNum;
#endif

#if defined(__ppc__) && !defined(__LP64__)
#define SUPPORT_CFM
#endif

typedef enum {
    WebCFMExecutableType,
    WebMachOExecutableType
} WebExecutableType;

@interface WebNetscapePluginPackage : WebBasePluginPackage
{
    NPPluginFuncs pluginFuncs;
    NPNetscapeFuncs browserFuncs;
    
    uint16 pluginSize;
    uint16 pluginVersion;
    
    ResFileRefNum resourceRef;
    
    NPP_ShutdownProcPtr NP_Shutdown;

    BOOL isLoaded;
    BOOL needsUnload;
    unsigned int instanceCount;
     
#if USE(PLUGIN_HOST_PROCESS)
    cpu_type_t pluginHostArchitecture;
#endif
    
#ifdef SUPPORT_CFM
    BOOL isBundle;
    BOOL isCFM;
    CFragConnectionID connID;
#endif
}

// Netscape plug-in packages must be explicitly opened and closed by each plug-in instance.
// This is to protect Netscape plug-ins from being unloaded while they are in use.
- (void)open;
- (void)close;

- (WebExecutableType)executableType;
- (NPPluginFuncs *)pluginFuncs;

#if USE(PLUGIN_HOST_PROCESS)
- (cpu_type_t)pluginHostArchitecture;
#endif

@end
#endif
