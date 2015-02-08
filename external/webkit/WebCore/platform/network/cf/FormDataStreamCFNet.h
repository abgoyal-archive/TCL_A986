

#ifndef FormDataStreamCFNet_h_
#define FormDataStreamCFNet_h_

#include <CoreFoundation/CoreFoundation.h>
#include <wtf/Forward.h>

typedef struct _CFURLRequest* CFMutableURLRequestRef;
typedef const struct _CFURLRequest* CFURLRequestRef;

namespace WebCore {
    class FormData;
    void setHTTPBody(CFMutableURLRequestRef, PassRefPtr<FormData>);
    PassRefPtr<FormData> httpBodyFromRequest(CFURLRequestRef);
}

#endif FormDataStreamCFNet_h_
