

#include "config.h"
#include "TransformSource.h"

namespace WebCore {

TransformSource::TransformSource(const PlatformTransformSource& source)
    : m_source(source)
{
}

TransformSource::~TransformSource()
{
}

}
