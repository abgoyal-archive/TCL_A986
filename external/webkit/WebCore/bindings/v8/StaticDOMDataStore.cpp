

#include "config.h"
#include "StaticDOMDataStore.h"

namespace WebCore {

StaticDOMDataStore::StaticDOMDataStore(DOMData* domData)
    : DOMDataStore(domData)
    , m_staticDomNodeMap(&DOMDataStore::weakNodeCallback)
    , m_staticDomObjectMap(domData, &DOMDataStore::weakDOMObjectCallback)
    , m_staticActiveDomObjectMap(domData, &DOMDataStore::weakActiveDOMObjectCallback)
#if ENABLE(SVG)
    , m_staticDomSvgElementInstanceMap(domData, &DOMDataStore::weakSVGElementInstanceCallback)
    , m_staticDomSvgObjectWithContextMap(domData, &DOMDataStore::weakSVGObjectWithContextCallback)
#endif
{
    m_domNodeMap = &m_staticDomNodeMap;
    m_domObjectMap = &m_staticDomObjectMap;
    m_activeDomObjectMap = &m_staticActiveDomObjectMap;
#if ENABLE(SVG)
    m_domSvgElementInstanceMap = &m_staticDomSvgElementInstanceMap;
    m_domSvgObjectWithContextMap = &m_staticDomSvgObjectWithContextMap;
#endif
}

} // namespace WebCore
