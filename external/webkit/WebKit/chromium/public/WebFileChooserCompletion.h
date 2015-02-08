

#ifndef WebFileChooserCompletion_h
#define WebFileChooserCompletion_h

namespace WebKit {

class WebString;
template <typename T> class WebVector;

// Gets called back when WebViewClient finished choosing a file.
class WebFileChooserCompletion {
public:
    // Called with zero or more file names. Zero-lengthed vector means that
    // the user cancelled or that file choosing failed. The callback instance
    // is destroyed when this method is called.
    virtual void didChooseFile(const WebVector<WebString>& fileNames) = 0;
protected:
    virtual ~WebFileChooserCompletion() {}
};

} // namespace WebKit

#endif
