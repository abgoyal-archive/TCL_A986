

#ifndef PasteboardHelperGtk_h
#define PasteboardHelperGtk_h


#include "Frame.h"
#include "PasteboardHelper.h"

#include <gtk/gtk.h>

using namespace WebCore;

namespace WebKit {

class PasteboardHelperGtk : public PasteboardHelper {
public:
    PasteboardHelperGtk();
    ~PasteboardHelperGtk();
    virtual GtkClipboard* getCurrentTarget(Frame*) const;
    virtual GtkClipboard* getClipboard(Frame*) const;
    virtual GtkClipboard* getPrimary(Frame*) const;
    virtual GtkTargetList* targetList() const;
    virtual gint getWebViewTargetInfoHtml() const;

    void writeClipboardContents(GtkClipboard* clipboard, gpointer data = 0);

private:
    GtkTargetList* m_targetList;
};

}

#endif // PasteboardHelperGtk_h
