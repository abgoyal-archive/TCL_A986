

#ifndef WebFileChooserCompletionImpl_h
#define WebFileChooserCompletionImpl_h

// FIXME: These relative paths are a temporary hack to support using this
// header from webkit/glue.
#include "../public/WebFileChooserCompletion.h"
#include "../public/WebString.h"
#include "../public/WebVector.h"

#include "FileChooser.h"

#include <wtf/PassRefPtr.h>

using WebKit::WebFileChooserCompletion;
using WebKit::WebString;
using WebKit::WebVector;

namespace WebKit {

class WebFileChooserCompletionImpl : public WebFileChooserCompletion {
public:
    WebFileChooserCompletionImpl(PassRefPtr<WebCore::FileChooser> chooser);
    ~WebFileChooserCompletionImpl();
    virtual void didChooseFile(const WebVector<WebString>& fileNames);
private:
    RefPtr<WebCore::FileChooser> m_fileChooser;
};

} // namespace WebKit

#endif
