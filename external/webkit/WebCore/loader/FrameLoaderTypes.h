

#ifndef FrameLoaderTypes_h
#define FrameLoaderTypes_h

namespace WebCore {

    enum FrameState {
        FrameStateProvisional,
        // This state indicates we are ready to commit to a page,
        // which means the view will transition to use the new data source.
        FrameStateCommittedPage,
        FrameStateComplete
    };

    enum PolicyAction {
        PolicyUse,
        PolicyDownload,
        PolicyIgnore,
    };

    // NOTE: Keep in sync with WebKit/mac/WebView/WebFramePrivate.h and WebKit/win/Interfaces/IWebFramePrivate.idl
    enum FrameLoadType {
        FrameLoadTypeStandard,
        FrameLoadTypeBack,
        FrameLoadTypeForward,
        FrameLoadTypeIndexedBackForward, // a multi-item hop in the backforward list
        FrameLoadTypeReload,
        // Skipped value: 'FrameLoadTypeReloadAllowingStaleData', still present in mac/win public API. Ready to be reused
        FrameLoadTypeSame = FrameLoadTypeReload + 2, // user loads same URL again (but not reload button)
        FrameLoadTypeRedirectWithLockedBackForwardList, // FIXME: Merge "lockBackForwardList", "lockHistory", "quickRedirect" and "clientRedirect" into a single concept of redirect.
        FrameLoadTypeReplace,
        FrameLoadTypeReloadFromOrigin,
        FrameLoadTypeBackWMLDeckNotAccessible
    };

    enum NavigationType {
        NavigationTypeLinkClicked,
        NavigationTypeFormSubmitted,
        NavigationTypeBackForward,
        NavigationTypeReload,
        NavigationTypeFormResubmitted,
        NavigationTypeOther
    };

    enum DatabasePolicy {
        DatabasePolicyStop,    // The database thread should be stopped and database connections closed.
        DatabasePolicyContinue
    };

    enum ObjectContentType {
        ObjectContentNone,
        ObjectContentImage,
        ObjectContentFrame,
        ObjectContentNetscapePlugin,
        ObjectContentOtherPlugin
    };
    
    enum UnloadEventPolicy {
        UnloadEventPolicyNone,
        UnloadEventPolicyUnloadOnly,
        UnloadEventPolicyUnloadAndPageHide
    };

    enum ReferrerPolicy {
        SendReferrer,
        NoReferrer
    };
    
    enum SandboxFlag {
        SandboxNone = 0,
        SandboxNavigation = 1,
        SandboxPlugins = 1 << 1,
        SandboxOrigin = 1 << 2,
        SandboxForms = 1 << 3,
        SandboxScripts = 1 << 4,
        SandboxAll = -1 // Mask with all bits set to 1.
    };
    
    enum SecurityCheckPolicy {
        SkipSecurityCheck,
        DoSecurityCheck
    };

    typedef int SandboxFlags;
}

#endif
