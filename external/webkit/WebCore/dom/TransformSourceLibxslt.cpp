

#include "config.h"

#if ENABLE(XSLT)
#include "TransformSource.h"

#include <libxml/tree.h>

namespace WebCore {

TransformSource::TransformSource(const PlatformTransformSource& source)
    : m_source(source)
{
}

TransformSource::~TransformSource()
{
    xmlFreeDoc((xmlDocPtr)m_source);
}

}

#endif
