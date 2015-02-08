

#ifndef SVGElementRareData_h
#define SVGElementRareData_h

#include <wtf/HashSet.h>
#include <wtf/Noncopyable.h>
#include <wtf/StdLibExtras.h>

namespace WebCore {

class CSSCursorImageValue;
class SVGCursorElement;
class SVGElement;
class SVGElementInstance;

class SVGElementRareData : public Noncopyable {
public:
    SVGElementRareData()
        : m_cursorElement(0)
        , m_cursorImageValue(0)
        , m_instancesUpdatesBlocked(false)
    {
    }

    typedef HashMap<const SVGElement*, SVGElementRareData*> SVGElementRareDataMap;

    static SVGElementRareDataMap& rareDataMap()
    {
        DEFINE_STATIC_LOCAL(SVGElementRareDataMap, rareDataMap, ());
        return rareDataMap;
    }

    static SVGElementRareData* rareDataFromMap(const SVGElement* element)
    {
        return rareDataMap().get(element);
    }

    HashSet<SVGElementInstance*>& elementInstances() { return m_elementInstances; }
    const HashSet<SVGElementInstance*>& elementInstances() const { return m_elementInstances; }

    bool instanceUpdatesBlocked() const { return m_instancesUpdatesBlocked; }
    void setInstanceUpdatesBlocked(bool value) { m_instancesUpdatesBlocked = value; }

    SVGCursorElement* cursorElement() const { return m_cursorElement; }
    void setCursorElement(SVGCursorElement* cursorElement) { m_cursorElement = cursorElement; }

    CSSCursorImageValue* cursorImageValue() const { return m_cursorImageValue; }
    void setCursorImageValue(CSSCursorImageValue* cursorImageValue) { m_cursorImageValue = cursorImageValue; }

private:
    HashSet<SVGElementInstance*> m_elementInstances;
    SVGCursorElement* m_cursorElement;
    CSSCursorImageValue* m_cursorImageValue;
    bool m_instancesUpdatesBlocked : 1;
};

}

#endif
