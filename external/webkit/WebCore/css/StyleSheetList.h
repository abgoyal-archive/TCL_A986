

#ifndef StyleSheetList_h
#define StyleSheetList_h

#include <wtf/RefCounted.h>
#include <wtf/PassRefPtr.h>
#include <wtf/Vector.h>

namespace WebCore {

class Document;
class HTMLStyleElement;
class StyleSheet;
class String;

typedef Vector<RefPtr<StyleSheet> > StyleSheetVector;

class StyleSheetList : public RefCounted<StyleSheetList> {
public:
    static PassRefPtr<StyleSheetList> create(Document* doc) { return adoptRef(new StyleSheetList(doc)); }
    ~StyleSheetList();

    void documentDestroyed();

    unsigned length() const;
    StyleSheet* item(unsigned index);

    HTMLStyleElement* getNamedItem(const String&) const;

    void swap(StyleSheetVector& sheets)
    {
        m_sheets.swap(sheets);
    }
    
private:
    StyleSheetList(Document*);

    Document* m_doc;
    StyleSheetVector m_sheets;
};

} // namespace WebCore

#endif // StyleSheetList_h
