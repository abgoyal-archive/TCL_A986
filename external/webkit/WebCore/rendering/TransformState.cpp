

#include "config.h"
#include "TransformState.h"

namespace WebCore {

void TransformState::move(int x, int y, TransformAccumulation accumulate)
{
    if (m_accumulatingTransform && m_accumulatedTransform) {
        // If we're accumulating into an existing transform, apply the translation.
        if (m_direction == ApplyTransformDirection)
            m_accumulatedTransform->translateRight(x, y);
        else
            m_accumulatedTransform->translate(-x, -y);  // We're unapplying, so negate
        
        // Then flatten if necessary.
        if (accumulate == FlattenTransform)
            flatten();
    } else {
        // Just move the point and, optionally, the quad.
        m_lastPlanarPoint.move(x, y);
        if (m_mapQuad)
            m_lastPlanarQuad.move(x, y);
    }
    m_accumulatingTransform = accumulate == AccumulateTransform;
}

// FIXME: We transform AffineTransform to TransformationMatrix. This is rather inefficient.
void TransformState::applyTransform(const AffineTransform& transformFromContainer, TransformAccumulation accumulate)
{
    applyTransform(transformFromContainer.toTransformationMatrix(), accumulate);
}

void TransformState::applyTransform(const TransformationMatrix& transformFromContainer, TransformAccumulation accumulate)
{
    // If we have an accumulated transform from last time, multiply in this transform
    if (m_accumulatedTransform) {
        if (m_direction == ApplyTransformDirection)
            m_accumulatedTransform->multiply(transformFromContainer);
        else
            m_accumulatedTransform->multLeft(transformFromContainer);
    } else if (accumulate == AccumulateTransform) {
        // Make one if we started to accumulate
        m_accumulatedTransform.set(new TransformationMatrix(transformFromContainer));
    }
    
    if (accumulate == FlattenTransform) {
        const TransformationMatrix* finalTransform = m_accumulatedTransform ? m_accumulatedTransform.get() : &transformFromContainer;
        flattenWithTransform(*finalTransform);
    }
    m_accumulatingTransform = accumulate == AccumulateTransform;
}

void TransformState::flatten()
{
    if (!m_accumulatedTransform) {
        m_accumulatingTransform = false;
        return;
    }
    
    flattenWithTransform(*m_accumulatedTransform);
}

FloatPoint TransformState::mappedPoint() const
{
    if (!m_accumulatedTransform)
        return m_lastPlanarPoint;

    if (m_direction == ApplyTransformDirection)
        return m_accumulatedTransform->mapPoint(m_lastPlanarPoint);

    return m_accumulatedTransform->inverse().projectPoint(m_lastPlanarPoint);
}

FloatQuad TransformState::mappedQuad() const
{
    if (!m_accumulatedTransform)
        return m_lastPlanarQuad;

    if (m_direction == ApplyTransformDirection)
        return m_accumulatedTransform->mapQuad(m_lastPlanarQuad);

    return m_accumulatedTransform->inverse().projectQuad(m_lastPlanarQuad);
}

void TransformState::flattenWithTransform(const TransformationMatrix& t)
{
    if (m_direction == ApplyTransformDirection) {
        m_lastPlanarPoint = t.mapPoint(m_lastPlanarPoint);
        if (m_mapQuad)
            m_lastPlanarQuad = t.mapQuad(m_lastPlanarQuad);
    } else {
        TransformationMatrix inverseTransform = t.inverse();
        m_lastPlanarPoint = inverseTransform.projectPoint(m_lastPlanarPoint);
        if (m_mapQuad)
            m_lastPlanarQuad = inverseTransform.projectQuad(m_lastPlanarQuad);
    }

    // We could throw away m_accumulatedTransform if we wanted to here, but that
    // would cause thrash when traversing hierarchies with alternating
    // preserve-3d and flat elements.
    if (m_accumulatedTransform)
        m_accumulatedTransform->makeIdentity();
    m_accumulatingTransform = false;
}

// HitTestingTransformState methods
void HitTestingTransformState::translate(int x, int y, TransformAccumulation accumulate)
{
    m_accumulatedTransform.translate(x, y);    
    if (accumulate == FlattenTransform)
        flattenWithTransform(m_accumulatedTransform);

    m_accumulatingTransform = accumulate == AccumulateTransform;
}

void HitTestingTransformState::applyTransform(const TransformationMatrix& transformFromContainer, TransformAccumulation accumulate)
{
    m_accumulatedTransform.multLeft(transformFromContainer);    
    if (accumulate == FlattenTransform)
        flattenWithTransform(m_accumulatedTransform);

    m_accumulatingTransform = accumulate == AccumulateTransform;
}

void HitTestingTransformState::flatten()
{
    flattenWithTransform(m_accumulatedTransform);
}

void HitTestingTransformState::flattenWithTransform(const TransformationMatrix& t)
{
    TransformationMatrix inverseTransform = t.inverse();
    m_lastPlanarPoint = inverseTransform.projectPoint(m_lastPlanarPoint);
    m_lastPlanarQuad = inverseTransform.projectQuad(m_lastPlanarQuad);

    m_accumulatedTransform.makeIdentity();
    m_accumulatingTransform = false;
}

FloatPoint HitTestingTransformState::mappedPoint() const
{
    return m_accumulatedTransform.inverse().projectPoint(m_lastPlanarPoint);
}

FloatQuad HitTestingTransformState::mappedQuad() const
{
    return m_accumulatedTransform.inverse().projectQuad(m_lastPlanarQuad);
}

} // namespace WebCore
