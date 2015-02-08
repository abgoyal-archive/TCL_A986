

#ifndef SVGAnimatedPropertySynchronizer_h
#define SVGAnimatedPropertySynchronizer_h

#include "SVGAnimatedPropertyTraits.h"

#if ENABLE(SVG)
namespace WebCore {

class SVGElement;

// GetOwnerElementForType implementation
template<typename OwnerType, bool isDerivedFromSVGElement>
struct GetOwnerElementForType;

template<typename OwnerType>
struct GetOwnerElementForType<OwnerType, true> : public Noncopyable {
    static SVGElement* ownerElement(OwnerType* type)
    {
        return type;
    }
};

template<typename OwnerType>
struct GetOwnerElementForType<OwnerType, false> : public Noncopyable {    
    static SVGElement* ownerElement(OwnerType* type)
    {
        SVGElement* context = type->contextElement();
        ASSERT(context);
        return context;
    }
};

// IsDerivedFromSVGElement implementation
template<typename OwnerType>
struct IsDerivedFromSVGElement : public Noncopyable {
    static const bool value = true;
};

class SVGViewSpec;
template<>
struct IsDerivedFromSVGElement<SVGViewSpec> : public Noncopyable {
    static const bool value = false;
};

// Helper template used for synchronizing SVG <-> XML properties
template<bool isDerivedFromSVGElement>
struct SVGAnimatedPropertySynchronizer {
    static void synchronize(SVGElement*, const QualifiedName&, const AtomicString&);
};

template<>
struct SVGAnimatedPropertySynchronizer<true> {
    static void synchronize(SVGElement* ownerElement, const QualifiedName& attrName, const AtomicString& value)
    {
        NamedNodeMap* namedAttrMap = ownerElement->attributes(false); 
        Attribute* old = namedAttrMap->getAttributeItem(attrName);
        if (old && value.isNull()) 
            namedAttrMap->removeAttribute(old->name()); 
        else if (!old && !value.isNull()) 
            namedAttrMap->addAttribute(ownerElement->createAttribute(attrName, value));
        else if (old && !value.isNull()) 
            old->setValue(value); 
    }
};

template<>
struct SVGAnimatedPropertySynchronizer<false> {
    static void synchronize(SVGElement*, const QualifiedName&, const AtomicString&)
    {
        // no-op, for types not inheriting from Element, thus nothing to synchronize
    }
};

};

#endif
#endif
