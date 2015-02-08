

#include "config.h"
#include "ScopedDOMDataStore.h"

namespace WebCore {

ScopedDOMDataStore::ScopedDOMDataStore(DOMData* domData)
    : DOMDataStore(domData)
{
    m_domNodeMap = new InternalDOMWrapperMap<Node>(domData, &DOMDataStore::weakNodeCallback);
    m_domObjectMap = new InternalDOMWrapperMap<void>(domData, &DOMDataStore::weakDOMObjectCallback);
    m_activeDomObjectMap = new InternalDOMWrapperMap<void>(domData, &DOMDataStore::weakActiveDOMObjectCallback);
#if ENABLE(SVG)
    m_domSvgElementInstanceMap = new InternalDOMWrapperMap<SVGElementInstance>(domData, &DOMDataStore::weakSVGElementInstanceCallback);
    m_domSvgObjectWithContextMap = new InternalDOMWrapperMap<void>(domData, &DOMDataStore::weakSVGObjectWithContextCallback);
#endif
}

ScopedDOMDataStore::~ScopedDOMDataStore()
{
    delete m_domNodeMap;
    delete m_domObjectMap;
    delete m_activeDomObjectMap;
#if ENABLE(SVG)
    delete m_domSvgElementInstanceMap;
    delete m_domSvgObjectWithContextMap;
#endif
}

} // namespace WebCore
