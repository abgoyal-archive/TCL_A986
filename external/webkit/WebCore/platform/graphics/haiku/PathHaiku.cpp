

#include "config.h"
#include "Path.h"

#include "AffineTransform.h"
#include "FloatRect.h"
#include "NotImplemented.h"
#include "PlatformString.h"
#include <Region.h>


namespace WebCore {

Path::Path()
    : m_path(new BRegion())
{
}

Path::~Path()
{
    delete m_path;
}

Path::Path(const Path& other)
    : m_path(new BRegion(*other.platformPath()))
{
}

Path& Path::operator=(const Path& other)
{
    if (&other != this)
        m_path = other.platformPath();

    return *this;
}

bool Path::hasCurrentPoint() const
{
    return !isEmpty();
}

bool Path::contains(const FloatPoint& point, WindRule rule) const
{
    return m_path->Contains(point);
}

void Path::translate(const FloatSize& size)
{
    notImplemented();
}

FloatRect Path::boundingRect() const
{
    return m_path->Frame();
}

void Path::moveTo(const FloatPoint& point)
{
    // FIXME: Use OffsetBy?
    notImplemented();
}

void Path::addLineTo(const FloatPoint& p)
{
    notImplemented();
}

void Path::addQuadCurveTo(const FloatPoint& cp, const FloatPoint& p)
{
    notImplemented();
}

void Path::addBezierCurveTo(const FloatPoint& cp1, const FloatPoint& cp2, const FloatPoint& p)
{
    notImplemented();
}

void Path::addArcTo(const FloatPoint& p1, const FloatPoint& p2, float radius)
{
    notImplemented();
}

void Path::closeSubpath()
{
    notImplemented();
}

void Path::addArc(const FloatPoint& p, float r, float sar, float ear, bool anticlockwise)
{
    notImplemented();
}

void Path::addRect(const FloatRect& r)
{
    m_path->Include(r);
}

void Path::addEllipse(const FloatRect& r)
{
    notImplemented();
}

void Path::clear()
{
    m_path->MakeEmpty();
}

bool Path::isEmpty() const
{
    return !m_path->Frame().IsValid();
}

String Path::debugString() const
{
    notImplemented();
    return String();
}

void Path::apply(void* info, PathApplierFunction function) const
{
    notImplemented();
}

void Path::transform(const AffineTransform& transform)
{
    notImplemented();
}

FloatRect Path::strokeBoundingRect(StrokeStyleApplier* applier)
{
    notImplemented();
    return FloatRect();
}

} // namespace WebCore

