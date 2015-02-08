

#include "config.h"

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGFESpecularLighting.h"
#include "SVGRenderTreeAsText.h"
#include "Filter.h"

namespace WebCore {

FESpecularLighting::FESpecularLighting(FilterEffect* in, const Color& lightingColor, const float& surfaceScale,
    const float& specularConstant, const float& specularExponent, const float& kernelUnitLengthX,
    const float& kernelUnitLengthY, PassRefPtr<LightSource> lightSource)
    : FilterEffect()
    , m_in(in)
    , m_lightingColor(lightingColor)
    , m_surfaceScale(surfaceScale)
    , m_specularConstant(specularConstant)
    , m_specularExponent(specularExponent)
    , m_kernelUnitLengthX(kernelUnitLengthX)
    , m_kernelUnitLengthY(kernelUnitLengthY)
    , m_lightSource(lightSource)
{
}

PassRefPtr<FESpecularLighting> FESpecularLighting::create(FilterEffect* in, const Color& lightingColor,
    const float& surfaceScale, const float& specularConstant, const float& specularExponent,
    const float& kernelUnitLengthX, const float& kernelUnitLengthY, PassRefPtr<LightSource> lightSource)
{
    return adoptRef(new FESpecularLighting(in, lightingColor, surfaceScale, specularConstant, specularExponent,
        kernelUnitLengthX, kernelUnitLengthY, lightSource));
}

FESpecularLighting::~FESpecularLighting()
{
}

Color FESpecularLighting::lightingColor() const
{
    return m_lightingColor;
}

void FESpecularLighting::setLightingColor(const Color& lightingColor)
{
    m_lightingColor = lightingColor;
}

float FESpecularLighting::surfaceScale() const
{
    return m_surfaceScale;
}

void FESpecularLighting::setSurfaceScale(float surfaceScale)
{
    m_surfaceScale = surfaceScale;
}

float FESpecularLighting::specularConstant() const
{
    return m_specularConstant;
}

void FESpecularLighting::setSpecularConstant(float specularConstant)
{
    m_specularConstant = specularConstant;
}

float FESpecularLighting::specularExponent() const
{
    return m_specularExponent;
}

void FESpecularLighting::setSpecularExponent(float specularExponent)
{
    m_specularExponent = specularExponent;
}

float FESpecularLighting::kernelUnitLengthX() const
{
    return m_kernelUnitLengthX;
}

void FESpecularLighting::setKernelUnitLengthX(float kernelUnitLengthX)
{
    m_kernelUnitLengthX = kernelUnitLengthX;
}

float FESpecularLighting::kernelUnitLengthY() const
{
    return m_kernelUnitLengthY;
}

void FESpecularLighting::setKernelUnitLengthY(float kernelUnitLengthY)
{
    m_kernelUnitLengthY = kernelUnitLengthY;
}

const LightSource* FESpecularLighting::lightSource() const
{
    return m_lightSource.get();
}

void FESpecularLighting::setLightSource(PassRefPtr<LightSource> lightSource)
{
    m_lightSource = lightSource;
}

void FESpecularLighting::apply(Filter*)
{
}

void FESpecularLighting::dump()
{
}

TextStream& FESpecularLighting::externalRepresentation(TextStream& ts) const
{
    ts << "[type=SPECULAR-LIGHTING] ";
    FilterEffect::externalRepresentation(ts);
    ts << " [surface scale=" << m_surfaceScale << "]"
        << " [specual constant=" << m_specularConstant << "]"
        << " [specular exponent=" << m_specularExponent << "]";
    return ts;
}

} // namespace WebCore

#endif // ENABLE(SVG) && ENABLE(FILTERS)
