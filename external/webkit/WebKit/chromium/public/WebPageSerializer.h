

#ifndef WebPageSerializer_h
#define WebPageSerializer_h

#include "WebCommon.h"

namespace WebKit {
class WebFrame;
class WebPageSerializerClient;
class WebString;
class WebURL;
template <typename T> class WebVector;

// Get html data by serializing all frames of current page with lists
// which contain all resource links that have local copy.
class WebPageSerializer {
public:
    // This function will find out all frames and serialize them to HTML data.
    // We have a data buffer to temporary saving generated html data. We will
    // sequentially call WebPageSeriazlierClient once the data buffer is full.
    //
    // Return false means no available frame has been serialized, otherwise
    // return true.
    //
    // The parameter frame specifies which frame need to be serialized.
    // The parameter recursive specifies whether we need to
    // serialize all sub frames of the specified frame or not.
    // The parameter client specifies the pointer of interface
    // WebPageSerializerClient providing a sink interface to receive the
    // individual chunks of data to be saved.
    // The parameter links contain original URLs of all saved links.
    // The parameter localPaths contain corresponding local file paths of all
    // saved links, which matched with vector:links one by one.
    // The parameter localDirectoryName is relative path of directory which
    // contain all saved auxiliary files included all sub frames and resources.
    WEBKIT_API static bool serialize(WebFrame* frame,
                                     bool recursive,
                                     WebPageSerializerClient* client,
                                     const WebVector<WebURL>& links,
                                     const WebVector<WebString>& localPaths,
                                     const WebString& localDirectoryName);

    // FIXME: The following are here for unit testing purposes. Consider
    // changing the unit tests instead.

    // Generate the META for charset declaration.
    WEBKIT_API static WebString generateMetaCharsetDeclaration(const WebString& charset);
    // Generate the MOTW declaration.
    WEBKIT_API static WebString generateMarkOfTheWebDeclaration(const WebURL& url);
    // Generate the default base tag declaration.
    WEBKIT_API static WebString generateBaseTagDeclaration(const WebString& baseTarget);
};

}  // namespace WebKit

#endif
