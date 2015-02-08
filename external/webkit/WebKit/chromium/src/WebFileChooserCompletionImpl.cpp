

#include "config.h"
#include "WebFileChooserCompletionImpl.h"

namespace WebKit {

WebFileChooserCompletionImpl::WebFileChooserCompletionImpl(PassRefPtr<WebCore::FileChooser> chooser)
    : m_fileChooser(chooser)
{
}

WebFileChooserCompletionImpl::~WebFileChooserCompletionImpl()
{
}

void WebFileChooserCompletionImpl::didChooseFile(const WebVector<WebString>& fileNames)
{
    if (fileNames.size() == 1)
        m_fileChooser->chooseFile(fileNames[0]);
    else {
        // This clause handles a case of file_names.size()==0 too.
        Vector<WebCore::String> paths;
        for (size_t i = 0; i < fileNames.size(); ++i)
            paths.append(fileNames[i]);
        m_fileChooser->chooseFiles(paths);
    }
    // This object is no longer needed.
    delete this;
}

} // namespace WebKit
