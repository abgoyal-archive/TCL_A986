

#ifndef JSSVGContextCache_h
#define JSSVGContextCache_h

#if ENABLE(SVG)
#include "SVGElement.h"
#include <wtf/StdLibExtras.h>

namespace WebCore {

class DOMObject;

class JSSVGContextCache : public Noncopyable {
public:
    typedef HashMap<DOMObject*, SVGElement*> WrapperMap;

    static WrapperMap& wrapperMap()
    {
        DEFINE_STATIC_LOCAL(WrapperMap, s_wrapperMap, ());
        return s_wrapperMap;
    }

    static void addWrapper(DOMObject* wrapper, SVGElement* context)
    {
        ASSERT(wrapper);
        ASSERT(context);

        pair<WrapperMap::iterator, bool> result = wrapperMap().add(wrapper, context);
        if (result.second) {
            WrapperMap::iterator& it = result.first;
            ASSERT_UNUSED(it, it->second == context);
        }
    }

    static void forgetWrapper(DOMObject* wrapper)
    {
        ASSERT(wrapper);

        WrapperMap& map = wrapperMap();
        WrapperMap::iterator it = map.find(wrapper);
        if (it == map.end())
            return;

        map.remove(it);
    }

    static void propagateSVGDOMChange(DOMObject* wrapper, const QualifiedName& attributeName)
    {
        WrapperMap& map = wrapperMap();
        WrapperMap::iterator it = map.find(wrapper);
        if (it == map.end())
            return;

        SVGElement* context = it->second;
        ASSERT(context);

        context->svgAttributeChanged(attributeName);
    }

    static SVGElement* svgContextForDOMObject(DOMObject* wrapper)
    {
        ASSERT(wrapper);

        WrapperMap& map = wrapperMap();
        WrapperMap::iterator it = map.find(wrapper);
        if (it == map.end())
            return 0;

        SVGElement* context = it->second;
        ASSERT(context);
        return context;
    }

};

}

#endif
#endif
