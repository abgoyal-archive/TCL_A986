
 
#ifndef WXWEBFRAMEPRIVATE_H
#define WXWEBFRAMEPRIVATE_H

#include "config.h"
#include "Frame.h"
#include "HTMLFrameOwnerElement.h"
#include "KURL.h"

class WebFramePrivate {

public:
    WebFramePrivate() :
            frame(0)
    {}
    
    WebCore::Frame* frame;
};

class WebViewFrameData
{
public:
    WebCore::KURL url;
    WebCore::String name;
    WebCore::HTMLFrameOwnerElement* ownerElement;
    
    WebCore::String referrer;
    bool allowsScrolling;
    int marginWidth;
    int marginHeight;    
};

#endif
