

#ifndef SVGScriptElement_h
#define SVGScriptElement_h

#if ENABLE(SVG)
#include "ScriptElement.h"
#include "SVGElement.h"
#include "SVGURIReference.h"
#include "SVGExternalResourcesRequired.h"

namespace WebCore {

    class SVGScriptElement : public SVGElement
                           , public SVGURIReference
                           , public SVGExternalResourcesRequired
                           , public ScriptElement {
    public:
        SVGScriptElement(const QualifiedName&, Document*, bool createdByParser);
        virtual ~SVGScriptElement();

        virtual String scriptContent() const;

        virtual void parseMappedAttribute(MappedAttribute*);
        virtual void insertedIntoDocument();
        virtual void removedFromDocument();
        virtual void childrenChanged(bool changedByParser = false, Node* beforeChange = 0, Node* afterChange = 0, int childCountDelta = 0);

        virtual void svgAttributeChanged(const QualifiedName&);
        virtual void synchronizeProperty(const QualifiedName&);
        virtual bool isURLAttribute(Attribute*) const;
        virtual void finishParsingChildren();

        String type() const;
        void setType(const String&);

        virtual String scriptCharset() const;

        virtual void addSubresourceAttributeURLs(ListHashSet<KURL>&) const;

        virtual bool shouldExecuteAsJavaScript() const { return false; }

    protected:
        virtual bool haveLoadedRequiredResources();

        virtual String sourceAttributeValue() const;
        virtual String charsetAttributeValue() const;
        virtual String typeAttributeValue() const;
        virtual String languageAttributeValue() const;
        virtual String forAttributeValue() const;

        virtual void dispatchLoadEvent();
        virtual void dispatchErrorEvent();

    private:
        // SVGURIReference
        DECLARE_ANIMATED_PROPERTY(SVGScriptElement, XLinkNames::hrefAttr, String, Href, href)

        // SVGExternalResourcesRequired
        DECLARE_ANIMATED_PROPERTY(SVGScriptElement, SVGNames::externalResourcesRequiredAttr, bool, ExternalResourcesRequired, externalResourcesRequired)

    private:
        ScriptElementData m_data;
        String m_type;
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
