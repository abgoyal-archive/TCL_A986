

#ifndef WebNavigationType_h
#define WebNavigationType_h

namespace WebKit {

enum WebNavigationType {
    WebNavigationTypeLinkClicked,
    WebNavigationTypeFormSubmitted,
    WebNavigationTypeBackForward,
    WebNavigationTypeReload,
    WebNavigationTypeFormResubmitted,
    WebNavigationTypeOther
};

} // namespace WebKit

#endif
