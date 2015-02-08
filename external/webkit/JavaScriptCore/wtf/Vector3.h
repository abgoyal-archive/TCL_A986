

#ifndef Vector3_h
#define Vector3_h

#include <math.h>

namespace WebCore {

class Vector3 {
public:
    Vector3()
        : m_x(0.0)
        , m_y(0.0)
        , m_z(0.0)
    {
    }

    Vector3(double x, double y, double z)
        : m_x(x)
        , m_y(y)
        , m_z(z)
    {
    }

    Vector3(const float p[3])
        : m_x(p[0])
        , m_y(p[1])
        , m_z(p[2])
    {
    }

    Vector3(const double p[3])
        : m_x(p[0])
        , m_y(p[1])
        , m_z(p[2])
    {
    }

    double abs() const
    {
        return sqrt(m_x * m_x + m_y * m_y + m_z * m_z);
    }

    bool isZero() const
    {
        return !m_x && !m_y && !m_z;
    }

    void normalize()
    {
        double absValue = abs();
        if (!absValue)
            return;

        double k = 1.0 / absValue;
        m_x *= k;
        m_y *= k;
        m_z *= k;
    }

    double x() const { return m_x; }
    double y() const { return m_y; }
    double z() const { return m_z; }

private:
    double m_x;
    double m_y;
    double m_z;
};

inline Vector3 operator+(const Vector3& v1, const Vector3& v2)
{
    return Vector3(v1.x() + v2.x(), v1.y() + v2.y(), v1.z() + v2.z());
}

inline Vector3 operator-(const Vector3& v1, const Vector3& v2)
{
    return Vector3(v1.x() - v2.x(), v1.y() - v2.y(), v1.z() - v2.z());
}

inline Vector3 operator*(double k, const Vector3& v)
{
    return Vector3(k * v.x(), k * v.y(), k * v.z());
}

inline Vector3 operator*(const Vector3& v, double k)
{
    return Vector3(k * v.x(), k * v.y(), k * v.z());
}

inline double dot(const Vector3& v1, const Vector3& v2)
{
    return v1.x() * v2.x() + v1.y() * v2.y() + v1.z() * v2.z();
}

inline Vector3 cross(const Vector3& v1, const Vector3& v2)
{
    double x3 = v1.y() * v2.z() - v1.z() * v2.y();
    double y3 = v1.z() * v2.x() - v1.x() * v2.z();
    double z3 = v1.x() * v2.y() - v1.y() * v2.x();
    return Vector3(x3, y3, z3);
}

inline double distance(const Vector3& v1, const Vector3& v2)
{
    return (v1 - v2).abs();
}

} // WebCore

#endif // Vector3_h
