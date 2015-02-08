

#ifndef SVGLangSpace_h
#define SVGLangSpace_h

#if ENABLE(SVG)
#include "AtomicString.h"

namespace WebCore {

    class MappedAttribute;
    class QualifiedName;

    class SVGLangSpace {
    public:
        SVGLangSpace();
        virtual ~SVGLangSpace();

        const AtomicString& xmllang() const;
        void setXmllang(const AtomicString& xmlLang);

        const AtomicString& xmlspace() const;
        void setXmlspace(const AtomicString& xmlSpace);

        bool parseMappedAttribute(MappedAttribute*);
        bool isKnownAttribute(const QualifiedName&);

    private:
        AtomicString m_lang;
        AtomicString m_space;
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif // SVGLangSpace_h
