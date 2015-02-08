

#include "config.h"
#include "VGUtils.h"

#include "FloatRect.h"
#include "TransformationMatrix.h"

namespace WebCore {

VGMatrix::VGMatrix(const VGfloat data[9])
{
    m_data[0] = data[0];
    m_data[1] = data[1];
    m_data[2] = data[2];
    m_data[3] = data[3];
    m_data[4] = data[4];
    m_data[5] = data[5];
    m_data[6] = data[6];
    m_data[7] = data[7];
    m_data[8] = data[8];
}

VGMatrix::VGMatrix(const TransformationMatrix& matrix)
{
    m_data[0] = matrix.m11();
    m_data[1] = matrix.m12();
    m_data[2] = matrix.m14();
    m_data[3] = matrix.m21();
    m_data[4] = matrix.m22();
    m_data[5] = matrix.m24();
    m_data[6] = matrix.m41();
    m_data[7] = matrix.m42();
    m_data[8] = matrix.m44();
}

VGMatrix::operator TransformationMatrix() const
{
    TransformationMatrix matrix;
    matrix.setM11(m_data[0]);
    matrix.setM12(m_data[1]);
    matrix.setM14(m_data[2]);
    matrix.setM21(m_data[3]);
    matrix.setM22(m_data[4]);
    matrix.setM24(m_data[5]);
    matrix.setM41(m_data[6]);
    matrix.setM42(m_data[7]);
    matrix.setM44(m_data[8]);
    return matrix;
}

TransformationMatrix::operator VGMatrix() const
{
    return VGMatrix(*this);
}

VGRect::VGRect(const VGfloat data[4])
{
    m_data[0] = data[0];
    m_data[1] = data[1];
    m_data[2] = data[2];
    m_data[3] = data[3];
}

VGRect::VGRect(const FloatRect& rect)
{
    m_data[0] = rect.x();
    m_data[1] = rect.y();
    m_data[2] = rect.width();
    m_data[3] = rect.height();
}

VGRect::operator FloatRect() const
{
    return FloatRect(m_data[0], m_data[1], m_data[2], m_data[3]);
}

FloatRect::operator VGRect() const
{
    return VGRect(*this);
}

}
