

#ifndef ScriptElement_h
#define ScriptElement_h

#include "CachedResourceClient.h"
#include "CachedResourceHandle.h"

namespace WebCore {

class CachedScript;
class Element;
class ScriptElementData;
class ScriptSourceCode;

class ScriptElement {
public:
    ScriptElement() { }
    virtual ~ScriptElement() { }

    virtual String scriptContent() const = 0;

    virtual String sourceAttributeValue() const = 0;
    virtual String charsetAttributeValue() const = 0;
    virtual String typeAttributeValue() const = 0;
    virtual String languageAttributeValue() const = 0;
    virtual String forAttributeValue() const = 0;

    virtual void dispatchLoadEvent() = 0;
    virtual void dispatchErrorEvent() = 0;

    // A charset for loading the script (may be overridden by HTTP headers or a BOM).
    virtual String scriptCharset() const = 0;

    virtual bool shouldExecuteAsJavaScript() const = 0;

protected:
    // Helper functions used by our parent classes.
    static void insertedIntoDocument(ScriptElementData&, const String& sourceUrl);
    static void removedFromDocument(ScriptElementData&);
    static void childrenChanged(ScriptElementData&);
    static void finishParsingChildren(ScriptElementData&, const String& sourceUrl);
    static void handleSourceAttribute(ScriptElementData&, const String& sourceUrl);
};

// HTML/SVGScriptElement hold this struct as member variable
// and pass it to the static helper functions in ScriptElement
class ScriptElementData : private CachedResourceClient {
public:
    ScriptElementData(ScriptElement*, Element*);
    virtual ~ScriptElementData();

    bool ignoresLoadRequest() const;
    bool shouldExecuteAsJavaScript() const;

    String scriptContent() const;
    String scriptCharset() const;

    Element* element() const { return m_element; }
    bool createdByParser() const { return m_createdByParser; }
    void setCreatedByParser(bool value) { m_createdByParser = value; }
    bool haveFiredLoadEvent() const { return m_firedLoad; }
    void setHaveFiredLoadEvent(bool firedLoad) { m_firedLoad = firedLoad; }

    void requestScript(const String& sourceUrl);
    void evaluateScript(const ScriptSourceCode&);
    void stopLoadRequest();

    void execute(CachedScript*);

private:
    virtual void notifyFinished(CachedResource*);

private:
    ScriptElement* m_scriptElement;
    Element* m_element;
    CachedResourceHandle<CachedScript> m_cachedScript;
    bool m_createdByParser;
    bool m_requested;
    bool m_evaluated;
    bool m_firedLoad;
};

ScriptElement* toScriptElement(Element*);

}

#endif
