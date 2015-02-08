

#ifndef WebNonCopyable_h
#define WebNonCopyable_h

namespace WebKit {

// A base class to extend from if you do not support copying.
class WebNonCopyable {
protected:
    WebNonCopyable() { }
    ~WebNonCopyable() { }

private:
    WebNonCopyable(const WebNonCopyable&);
    WebNonCopyable& operator=(const WebNonCopyable&);
};

} // namespace WebKit

#endif
