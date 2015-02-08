

#ifndef WebNavigationPolicy_h
#define WebNavigationPolicy_h

namespace WebKit {

enum WebNavigationPolicy {
    WebNavigationPolicyIgnore,
    WebNavigationPolicyDownload,
    WebNavigationPolicyCurrentTab,
    WebNavigationPolicyNewBackgroundTab,
    WebNavigationPolicyNewForegroundTab,
    WebNavigationPolicyNewWindow,
    WebNavigationPolicyNewPopup,
};

} // namespace WebKit

#endif
