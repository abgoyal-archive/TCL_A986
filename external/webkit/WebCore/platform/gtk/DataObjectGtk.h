

#ifndef DataObjectGtk_h
#define DataObjectGtk_h

#include "CString.h"
#include "FileList.h"
#include "KURL.h"
#include "Range.h"
#include "StringHash.h"
#include <wtf/RefCounted.h>
#include <wtf/gtk/GRefPtr.h>

typedef struct _GdkPixbuf GdkPixbuf;
typedef struct _GdkDragContext GdkDragContext;
typedef struct _GtkClipboard GtkClipboard;

namespace WebCore {

class DataObjectGtk : public RefCounted<DataObjectGtk> {
public:
    static PassRefPtr<DataObjectGtk> create()
    {
        return adoptRef(new DataObjectGtk());
    }

    Vector<KURL> uriList() { return m_uriList; }
    GdkPixbuf* image() { return m_image.get(); }
    void setRange(PassRefPtr<Range> newRange) { m_range = newRange; }
    void setURIList(const Vector<KURL>& newURIList) {  m_uriList = newURIList; }
    void setImage(GdkPixbuf* newImage) { m_image = newImage; }
    void setDragContext(GdkDragContext* newDragContext) { m_dragContext = newDragContext; }
    bool hasText() { return m_range || !m_text.isEmpty(); }
    bool hasMarkup() { return m_range || !m_markup.isEmpty(); }
    bool hasURIList() { return !m_uriList.isEmpty(); }
    bool hasImage() { return m_image; }
    GdkDragContext* dragContext() { return m_dragContext.get(); }

    String text();
    String markup();
    Vector<String> files();
    void setText(const String& newText);
    void setMarkup(const String& newMarkup);
    bool hasURL();
    String url();
    String urlLabel();
    void clear();

    static DataObjectGtk* forClipboard(GtkClipboard*);

private:
    String m_text;
    String m_markup;
    Vector<KURL> m_uriList;
    GRefPtr<GdkPixbuf> m_image;
    GRefPtr<GdkDragContext> m_dragContext;
    RefPtr<Range> m_range;
};

}

#endif // DataObjectGtk_h
