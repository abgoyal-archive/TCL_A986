

#include "config.h"

#if ENABLE(SVG)
#include "JSSVGPathSegList.h"

#include "Document.h"
#include "Frame.h"
#include "JSSVGContextCache.h"
#include "JSSVGPathSeg.h"
#include "SVGDocumentExtensions.h"
#include "SVGElement.h"
#include "SVGPathSegList.h"

#include <wtf/Assertions.h>

using namespace JSC;

namespace WebCore {

JSValue JSSVGPathSegList::clear(ExecState* exec, const ArgList&)
{
    ExceptionCode ec = 0;

    SVGPathSegList* list = impl();
    list->clear(ec);

    setDOMException(exec, ec);

    JSSVGContextCache::propagateSVGDOMChange(this, list->associatedAttributeName());
    return jsUndefined();
}

JSValue JSSVGPathSegList::initialize(ExecState* exec, const ArgList& args)
{
    ExceptionCode ec = 0;
    SVGPathSeg* newItem = toSVGPathSeg(args.at(0));

    SVGPathSegList* list = impl();

    SVGPathSeg* obj = WTF::getPtr(list->initialize(newItem, ec));
    SVGElement* context = JSSVGContextCache::svgContextForDOMObject(this);

    JSValue result = toJS(exec, globalObject(), obj, context);
    setDOMException(exec, ec);

    JSSVGContextCache::propagateSVGDOMChange(this, list->associatedAttributeName());
    return result;
}

JSValue JSSVGPathSegList::getItem(ExecState* exec, const ArgList& args)
{
    ExceptionCode ec = 0;

    bool indexOk;
    unsigned index = args.at(0).toInt32(exec, indexOk);
    if (!indexOk) {
        setDOMException(exec, TYPE_MISMATCH_ERR);
        return jsUndefined();
    }

    SVGPathSegList* list = impl();
    SVGPathSeg* obj = WTF::getPtr(list->getItem(index, ec));
    SVGElement* context = JSSVGContextCache::svgContextForDOMObject(this);

    JSValue result = toJS(exec, globalObject(), obj, context);
    setDOMException(exec, ec);
    return result;
}

JSValue JSSVGPathSegList::insertItemBefore(ExecState* exec, const ArgList& args)
{
    ExceptionCode ec = 0;
    SVGPathSeg* newItem = toSVGPathSeg(args.at(0));

    bool indexOk;
    unsigned index = args.at(1).toInt32(exec, indexOk);
    if (!indexOk) {
        setDOMException(exec, TYPE_MISMATCH_ERR);
        return jsUndefined();
    }

    SVGPathSegList* list = impl();
    SVGElement* context = JSSVGContextCache::svgContextForDOMObject(this);

    JSValue result = toJS(exec, globalObject(), WTF::getPtr(list->insertItemBefore(newItem, index, ec)), context);
    setDOMException(exec, ec);

    JSSVGContextCache::propagateSVGDOMChange(this, list->associatedAttributeName());
    return result;
}

JSValue JSSVGPathSegList::replaceItem(ExecState* exec, const ArgList& args)
{
    ExceptionCode ec = 0;
    SVGPathSeg* newItem = toSVGPathSeg(args.at(0));
    
    bool indexOk;
    unsigned index = args.at(1).toInt32(exec, indexOk);
    if (!indexOk) {
        setDOMException(exec, TYPE_MISMATCH_ERR);
        return jsUndefined();
    }

    SVGPathSegList* list = impl();
    SVGElement* context = JSSVGContextCache::svgContextForDOMObject(this);

    JSValue result = toJS(exec, globalObject(), WTF::getPtr(list->replaceItem(newItem, index, ec)), context);
    setDOMException(exec, ec);

    JSSVGContextCache::propagateSVGDOMChange(this, list->associatedAttributeName());
    return result;
}

JSValue JSSVGPathSegList::removeItem(ExecState* exec, const ArgList& args)
{
    ExceptionCode ec = 0;
    
    bool indexOk;
    unsigned index = args.at(0).toInt32(exec, indexOk);
    if (!indexOk) {
        setDOMException(exec, TYPE_MISMATCH_ERR);
        return jsUndefined();
    }

    SVGPathSegList* list = impl();

    RefPtr<SVGPathSeg> obj(list->removeItem(index, ec));
    SVGElement* context = JSSVGContextCache::svgContextForDOMObject(this);

    JSValue result = toJS(exec, globalObject(), obj.get(), context);
    setDOMException(exec, ec);

    JSSVGContextCache::propagateSVGDOMChange(this, list->associatedAttributeName());
    return result;
}

JSValue JSSVGPathSegList::appendItem(ExecState* exec, const ArgList& args)
{
    ExceptionCode ec = 0;
    SVGPathSeg* newItem = toSVGPathSeg(args.at(0));

    SVGPathSegList* list = impl();
    SVGElement* context = JSSVGContextCache::svgContextForDOMObject(this);

    JSValue result = toJS(exec, globalObject(), WTF::getPtr(list->appendItem(newItem, ec)), context);
    setDOMException(exec, ec);

    JSSVGContextCache::propagateSVGDOMChange(this, list->associatedAttributeName());
    return result;
}

}

#endif // ENABLE(SVG)
