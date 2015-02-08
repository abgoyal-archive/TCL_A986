

#ifndef HTMLDocument_h
#define HTMLDocument_h

#include "AtomicStringHash.h"
#include "CachedResourceClient.h"
#include "Document.h"
#include <wtf/HashCountedSet.h>

namespace WebCore {

class FrameView;
class HTMLElement;

class HTMLDocument : public Document, public CachedResourceClient {
public:
    static PassRefPtr<HTMLDocument> create(Frame* frame)
    {
        return adoptRef(new HTMLDocument(frame));
    }
    virtual ~HTMLDocument();

    int width();
    int height();

    String dir();
    void setDir(const String&);

    String designMode() const;
    void setDesignMode(const String&);

    String compatMode() const;

    Element* activeElement();
    bool hasFocus();

    String bgColor();
    void setBgColor(const String&);
    String fgColor();
    void setFgColor(const String&);
    String alinkColor();
    void setAlinkColor(const String&);
    String linkColor();
    void setLinkColor(const String&);
    String vlinkColor();
    void setVlinkColor(const String&);

    void clear();

    void captureEvents();
    void releaseEvents();

    void addNamedItem(const AtomicString& name);
    void removeNamedItem(const AtomicString& name);
    bool hasNamedItem(AtomicStringImpl* name);

    void addExtraNamedItem(const AtomicString& name);
    void removeExtraNamedItem(const AtomicString& name);
    bool hasExtraNamedItem(AtomicStringImpl* name);

protected:
    HTMLDocument(Frame*);

#ifdef ANDROID_INSTRUMENT
    // Overridden to resolve the ambiguous
    void* operator new(size_t size);
    void* operator new[](size_t size);
    void operator delete(void* p, size_t size);
    void operator delete[](void* p, size_t size);
#endif

private:
    virtual bool childAllowed(Node*);

    virtual PassRefPtr<Element> createElement(const AtomicString& tagName, ExceptionCode&);

    virtual bool isFrameSet() const;
    virtual Tokenizer* createTokenizer();
    virtual void determineParseMode();

    HashCountedSet<AtomicStringImpl*> m_namedItemCounts;
    HashCountedSet<AtomicStringImpl*> m_extraNamedItemCounts;
};

inline bool HTMLDocument::hasNamedItem(AtomicStringImpl* name)
{
    ASSERT(name);
    return m_namedItemCounts.contains(name);
}

inline bool HTMLDocument::hasExtraNamedItem(AtomicStringImpl* name)
{
    ASSERT(name);
    return m_extraNamedItemCounts.contains(name);
}

} // namespace WebCore

#endif // HTMLDocument_h
