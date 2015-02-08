

#include "config.h"
#include "CanvasGradient.h"

#include "CSSParser.h"
#include "ExceptionCode.h"

namespace WebCore {

CanvasGradient::CanvasGradient(const FloatPoint& p0, const FloatPoint& p1)
    : m_gradient(Gradient::create(p0, p1))
    , m_dashbardCompatibilityMode(false)
{
}

CanvasGradient::CanvasGradient(const FloatPoint& p0, float r0, const FloatPoint& p1, float r1)
    : m_gradient(Gradient::create(p0, r0, p1, r1))
    , m_dashbardCompatibilityMode(false)
{
}

void CanvasGradient::addColorStop(float value, const String& color, ExceptionCode& ec)
{
    if (!(value >= 0 && value <= 1.0f)) {
        ec = INDEX_SIZE_ERR;
        return;
    }

    RGBA32 rgba = 0;
    if (!CSSParser::parseColor(rgba, color)) {
        if (!m_dashbardCompatibilityMode)
            ec = SYNTAX_ERR;
        return;
    }

    m_gradient->addColorStop(value, Color(rgba));
}

} // namespace
