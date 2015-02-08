

#ifndef WebDataSource_h
#define WebDataSource_h

#include "WebCommon.h"
#include "WebNavigationType.h"

namespace WebKit {

class WebApplicationCacheHost;
class WebString;
class WebURL;
class WebURLRequest;
class WebURLResponse;
template <typename T> class WebVector;

class WebDataSource {
public:
    class ExtraData {
    public:
        virtual ~ExtraData() { }
    };

    // Returns the original request that resulted in this datasource.
    virtual const WebURLRequest& originalRequest() const = 0;

    // Returns the request corresponding to this datasource.  It may
    // include additional request headers added by WebKit that were not
    // present in the original request.  This request may also correspond
    // to a location specified by a redirect that was followed.
    virtual const WebURLRequest& request() const = 0;

    // Returns the response associated with this datasource.
    virtual const WebURLResponse& response() const = 0;

    // When this datasource was created as a result of WebFrame::loadData,
    // there may be an associated unreachableURL.
    virtual bool hasUnreachableURL() const = 0;
    virtual WebURL unreachableURL() const = 0;

    // Returns all redirects that occurred (both client and server) before
    // at last committing the current page.  This will contain one entry
    // for each intermediate URL, and one entry for the last URL (so if
    // there are no redirects, it will contain exactly the current URL, and
    // if there is one redirect, it will contain the source and destination
    // URL).
    virtual void redirectChain(WebVector<WebURL>&) const = 0;

    // Returns the title for the current page.
    virtual WebString pageTitle() const = 0;

    // The type of navigation that triggered the creation of this datasource.
    virtual WebNavigationType navigationType() const = 0;

    // The time in seconds (since the epoch) of the event that triggered
    // the creation of this datasource.  Returns 0 if unknown.
    virtual double triggeringEventTime() const = 0;

    // Extra data associated with this datasource.  If non-null, the extra
    // data pointer will be deleted when the datasource is destroyed.
    // Setting the extra data pointer will cause any existing non-null
    // extra data pointer to be deleted.
    virtual ExtraData* extraData() const = 0;
    virtual void setExtraData(ExtraData*) = 0;

    // The application cache host associated with this datasource.
    virtual WebApplicationCacheHost* applicationCacheHost() = 0;

protected:
    ~WebDataSource() { }
};

} // namespace WebKit

#endif
