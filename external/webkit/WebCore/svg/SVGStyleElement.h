

#ifndef SVGStyleElement_h
#define SVGStyleElement_h
#if ENABLE(SVG)

#include <SVGElement.h>
#include "SVGLangSpace.h"
#include "StyleElement.h"

namespace WebCore {

    class SVGStyleElement : public SVGElement,
                            public SVGLangSpace,
                            public StyleElement {
    public:
        SVGStyleElement(const QualifiedName&, Document*, bool createdByParser);

        // Derived from: 'Element'
        virtual void parseMappedAttribute(MappedAttribute*);
        virtual void insertedIntoDocument();
        virtual void removedFromDocument();
        virtual void childrenChanged(bool changedByParser = false, Node* beforeChange = 0, Node* afterChange = 0, int childCountDelta = 0);

        virtual void finishParsingChildren();

        virtual bool sheetLoaded();

        virtual const AtomicString& type() const;
        void setType(const AtomicString&, ExceptionCode&);

        virtual const AtomicString& media() const;
        void setMedia(const AtomicString&, ExceptionCode&);

        virtual String title() const;
        void setTitle(const AtomicString&, ExceptionCode&);

        StyleSheet* sheet();
        
    protected:
        bool m_createdByParser;
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif // SVGStyleElement_h

// vim:ts=4:noet
