

#ifndef FloatPoint3D_h
#define FloatPoint3D_h

namespace WebCore {

class FloatPoint;

class FloatPoint3D {
public:
    FloatPoint3D();
    FloatPoint3D(float x, float y, float z);
    FloatPoint3D(const FloatPoint&);

    float x() const { return m_x; }
    void setX(float x) { m_x = x; }

    float y() const { return m_y; }
    void setY(float y) { m_y = y; }

    float z() const { return m_z; }
    void setZ(float z) { m_z = z; }

    void normalize();

private:
    float m_x;
    float m_y;
    float m_z;
};

inline bool operator==(const FloatPoint3D& a, const FloatPoint3D& b)
{
    return a.x() == b.x() && a.y() == b.y() && a.z() == b.z();
}

inline bool operator!=(const FloatPoint3D& a, const FloatPoint3D& b)
{
    return a.x() != b.x() || a.y() != b.y() || a.z() != b.z();
}

} // namespace WebCore

#endif // FloatPoint3D_h
