
 
#ifndef EditorInsertAction_h
#define EditorInsertAction_h

namespace WebCore {

// This must be kept in sync with WebViewInsertAction defined in WebEditingDelegate.h
enum EditorInsertAction {
    EditorInsertActionTyped,
    EditorInsertActionPasted,
    EditorInsertActionDropped,
};

} // namespace

#endif
