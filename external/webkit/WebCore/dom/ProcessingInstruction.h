

#ifndef ProcessingInstruction_h
#define ProcessingInstruction_h

#include "CachedResourceClient.h"
#include "CachedResourceHandle.h"
#include "ContainerNode.h"

namespace WebCore {

class StyleSheet;
class CSSStyleSheet;

class ProcessingInstruction : public ContainerNode, private CachedResourceClient {
public:
    static PassRefPtr<ProcessingInstruction> create(Document*, const String& target, const String& data);
    virtual ~ProcessingInstruction();

    const String& target() const { return m_target; }
    const String& data() const { return m_data; }
    void setData(const String&, ExceptionCode&);

    void setCreatedByParser(bool createdByParser) { m_createdByParser = createdByParser; }

    virtual void finishParsingChildren();

    const String& localHref() const { return m_localHref; }
    StyleSheet* sheet() const { return m_sheet.get(); }
    void setCSSStyleSheet(PassRefPtr<CSSStyleSheet>);

#if ENABLE(XSLT)
    bool isXSL() const { return m_isXSL; }
#endif

private:
    ProcessingInstruction(Document*, const String& target, const String& data);

#ifdef ANDROID_INSTRUMENT
    // Overridden to resolve the ambiguous
    void* operator new(size_t size);
    void* operator new[](size_t size);
    void operator delete(void* p, size_t size);
    void operator delete[](void* p, size_t size);
#endif

    virtual String nodeName() const;
    virtual NodeType nodeType() const;
    virtual String nodeValue() const;
    virtual void setNodeValue(const String&, ExceptionCode&);
    virtual PassRefPtr<Node> cloneNode(bool deep);
    virtual bool childTypeAllowed(NodeType);
    virtual bool offsetInCharacters() const;
    virtual int maxCharacterOffset() const;

    virtual void insertedIntoDocument();
    virtual void removedFromDocument();

    void checkStyleSheet();
    virtual void setCSSStyleSheet(const String& href, const KURL& baseURL, const String& charset, const CachedCSSStyleSheet*);
#if ENABLE(XSLT)
    virtual void setXSLStyleSheet(const String& href, const KURL& baseURL, const String& sheet);
#endif

    bool isLoading() const;
    virtual bool sheetLoaded();

    virtual void addSubresourceAttributeURLs(ListHashSet<KURL>&) const;

    void parseStyleSheet(const String& sheet);

    String m_target;
    String m_data;
    String m_localHref;
    String m_title;
    String m_media;
    CachedResourceHandle<CachedResource> m_cachedSheet;
    RefPtr<StyleSheet> m_sheet;
    bool m_loading;
    bool m_alternate;
    bool m_createdByParser;
#if ENABLE(XSLT)
    bool m_isXSL;
#endif
};

} //namespace

#endif
