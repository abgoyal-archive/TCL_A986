

#ifndef CSSFontSelector_h
#define CSSFontSelector_h

#include "FontSelector.h"
#include "StringHash.h"
#include <wtf/HashMap.h>
#include <wtf/RefPtr.h>

namespace WebCore {

class AtomicString;
class CSSFontFace;
class CSSFontFaceRule;
class CSSSegmentedFontFace;
class Document;
class DocLoader;
class FontDescription;
class String;

class CSSFontSelector : public FontSelector {
public:
    static PassRefPtr<CSSFontSelector> create(Document* document)
    {
        return adoptRef(new CSSFontSelector(document));
    }
    virtual ~CSSFontSelector();

    virtual FontData* getFontData(const FontDescription& fontDescription, const AtomicString& familyName);
    
    void clearDocument() { m_document = 0; }

    void addFontFaceRule(const CSSFontFaceRule*);

    void fontLoaded();
    virtual void fontCacheInvalidated();

    bool isEmpty() const;

    DocLoader* docLoader() const;

private:
    CSSFontSelector(Document*);

    Document* m_document;
    HashMap<String, Vector<RefPtr<CSSFontFace> >*, CaseFoldingHash> m_fontFaces;
    HashMap<String, Vector<RefPtr<CSSFontFace> >*, CaseFoldingHash> m_locallyInstalledFontFaces;
    HashMap<String, HashMap<unsigned, RefPtr<CSSSegmentedFontFace> >*, CaseFoldingHash> m_fonts;
};

} // namespace WebCore

#endif // CSSFontSelector_h
