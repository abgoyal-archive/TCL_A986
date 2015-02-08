

#ifndef WebHTTPHeaderVisitor_h
#define WebHTTPHeaderVisitor_h

namespace WebKit {

class WebString;

class WebHTTPHeaderVisitor {
public:
    virtual void visitHeader(const WebString& name, const WebString& value) = 0;

protected:
    ~WebHTTPHeaderVisitor() { }
};

} // namespace WebKit

#endif
