

#ifndef WebContextMenuData_h
#define WebContextMenuData_h

#include "WebMenuItemInfo.h"
#include "WebPoint.h"
#include "WebString.h"
#include "WebURL.h"
#include "WebVector.h"

namespace WebKit {

// This struct is passed to WebViewClient::ShowContextMenu.
struct WebContextMenuData {
    enum MediaType {
        // No special node is in context.
        MediaTypeNone,
        // An image node is selected.
        MediaTypeImage,
        // A video node is selected.
        MediaTypeVideo,
        // An audio node is selected.
        MediaTypeAudio,
    };
    // The type of media the context menu is being invoked on.
    MediaType mediaType;

    // The x and y position of the mouse pointer (relative to the webview).
    WebPoint mousePosition;

    // The absolute URL of the link that is in context.
    WebURL linkURL;

    // The absolute URL of the image/video/audio that is in context.
    WebURL srcURL;

    // The absolute URL of the page in context.
    WebURL pageURL;

    // The absolute URL of the subframe in context.
    WebURL frameURL;

    // The encoding for the frame in context.
    WebString frameEncoding;

    enum MediaFlags {
        MediaNone = 0x0,
        MediaInError = 0x1,
        MediaPaused = 0x2,
        MediaMuted = 0x4,
        MediaLoop = 0x8,
        MediaCanSave = 0x10,
        MediaHasAudio = 0x20,
    };

    // Extra attributes describing media elements.
    int mediaFlags;

    // The raw text of the selection in context.
    WebString selectedText;

    // Whether spell checking is enabled.
    bool isSpellCheckingEnabled;

    // The editable (possibily) misspelled word.
    WebString misspelledWord;

    // Whether context is editable.
    bool isEditable;

    enum CheckableMenuItemFlags {
        CheckableMenuItemDisabled = 0x0,
        CheckableMenuItemEnabled = 0x1,
        CheckableMenuItemChecked = 0x2,
    };

    // Writing direction menu items - values are unions of
    // CheckableMenuItemFlags.
    // Currently only used on OS X.
    int writingDirectionDefault;
    int writingDirectionLeftToRight;
    int writingDirectionRightToLeft;

    enum EditFlags {
        CanDoNone = 0x0,
        CanUndo = 0x1,
        CanRedo = 0x2,
        CanCut = 0x4,
        CanCopy = 0x8,
        CanPaste = 0x10,
        CanDelete = 0x20,
        CanSelectAll = 0x40,
    };

    // Which edit operations are available in the context.
    int editFlags;

    // Security information for the context.
    WebCString securityInfo;

    // Custom context menu items provided by the WebCore internals.
    WebVector<WebMenuItemInfo> customItems;
};

} // namespace WebKit

#endif
