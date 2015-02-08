

#ifndef NamedMappedAttrMap_h
#define NamedMappedAttrMap_h

#include "NamedNodeMap.h"
#include "SpaceSplitString.h"

namespace WebCore {

class NamedMappedAttrMap : public NamedNodeMap {
public:
    static PassRefPtr<NamedMappedAttrMap> create(Element* element = 0) { return adoptRef(new NamedMappedAttrMap(element)); }

    void clearClass() { m_classNames.clear(); }
    void setClass(const String&);
    const SpaceSplitString& classNames() const { return m_classNames; }

    bool hasMappedAttributes() const { return m_mappedAttributeCount > 0; }
    void declRemoved() { m_mappedAttributeCount--; }
    void declAdded() { m_mappedAttributeCount++; }

    bool mapsEquivalent(const NamedMappedAttrMap*) const;

private:
    NamedMappedAttrMap(Element* element) : NamedNodeMap(element), m_mappedAttributeCount(0) { }

    virtual void clearAttributes();
    virtual bool isMappedAttributeMap() const;

    int declCount() const;

    SpaceSplitString m_classNames;
    int m_mappedAttributeCount;
};

} //namespace

#endif
