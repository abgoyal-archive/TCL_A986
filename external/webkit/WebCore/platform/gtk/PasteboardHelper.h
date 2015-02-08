

#ifndef PasteboardHelper_h
#define PasteboardHelper_h


#include "Frame.h"

typedef struct _GtkClipboard GtkClipboard;
typedef struct _GtkTargetList GtkTargetList;

namespace WebCore {

class PasteboardHelper {
public:
    virtual ~PasteboardHelper() {};

    virtual GtkClipboard* getCurrentTarget(Frame*) const = 0;
    virtual GtkClipboard* getClipboard(Frame*) const = 0;
    virtual GtkClipboard* getPrimary(Frame*) const = 0;
    virtual GtkTargetList* targetList() const = 0;
    virtual gint getWebViewTargetInfoHtml() const = 0;
};

}

#endif // PasteboardHelper_h
