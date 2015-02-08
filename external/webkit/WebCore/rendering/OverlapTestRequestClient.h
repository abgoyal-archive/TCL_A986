

#ifndef OverlapTestRequestClient_h
#define OverlapTestRequestClient_h

namespace WebCore {
    
class OverlapTestRequestClient {
public:
    virtual ~OverlapTestRequestClient() { }
    virtual void setOverlapTestResult(bool) = 0;
};

} // namespace WebCore

#endif // OverlapTestRequestClient_h
