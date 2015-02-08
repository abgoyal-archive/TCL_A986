

#ifndef SVGTests_h
#define SVGTests_h

#if ENABLE(SVG)
#include <wtf/RefPtr.h>

namespace WebCore {

    class MappedAttribute;
    class String;
    class SVGStringList;
    class QualifiedName;

    class SVGTests {
    public:
        SVGTests();
        virtual ~SVGTests();

        SVGStringList* requiredFeatures() const;
        SVGStringList* requiredExtensions() const;
        SVGStringList* systemLanguage() const;

        bool hasExtension(const String&) const;

        bool isValid() const;
        
        bool parseMappedAttribute(MappedAttribute*);
        bool isKnownAttribute(const QualifiedName&);

    private:
        mutable RefPtr<SVGStringList> m_features;
        mutable RefPtr<SVGStringList> m_extensions;
        mutable RefPtr<SVGStringList> m_systemLanguage;
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif // SVGTests_h
