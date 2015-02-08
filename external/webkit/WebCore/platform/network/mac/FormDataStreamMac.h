

#ifndef FormDataStreamMac_h
#define FormDataStreamMac_h

#include "FormData.h"

@class NSMutableURLRequest;

namespace WebCore {

    class FormData;
    class ResourceHandle;

    void setHTTPBody(NSMutableURLRequest *, PassRefPtr<FormData>);
    FormData* httpBodyFromStream(NSInputStream *);

    void associateStreamWithResourceHandle(NSInputStream *, ResourceHandle*);
    void disassociateStreamWithResourceHandle(NSInputStream *);

} // namespace WebCore

#endif // FormDataStreamMac_h
