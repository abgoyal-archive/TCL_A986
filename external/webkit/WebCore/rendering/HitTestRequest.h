

#ifndef HitTestRequest_h
#define HitTestRequest_h

namespace WebCore {

class HitTestRequest {
public:
    enum RequestType {
        ReadOnly = 0x1,
        Active = 0x2,
        MouseMove = 0x4,
        MouseUp = 0x8,
        IgnoreClipping = 0x10
    };

    HitTestRequest(int requestType)
        : m_requestType(requestType)
    {
    }

    bool readOnly() const { return m_requestType & ReadOnly; }
    bool active() const { return m_requestType & Active; }
    bool mouseMove() const { return m_requestType & MouseMove; }
    bool mouseUp() const { return m_requestType & MouseUp; }
    bool ignoreClipping() const { return m_requestType & IgnoreClipping; }

private:
    int m_requestType;
};

} // namespace WebCore

#endif // HitTestRequest_h
