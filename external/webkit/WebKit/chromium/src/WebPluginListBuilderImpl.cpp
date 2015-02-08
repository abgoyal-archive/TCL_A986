

#include "config.h"
#include "WebPluginListBuilderImpl.h"

#include "PluginData.h"
#include "WebString.h"
#include <wtf/Vector.h>

using namespace WebCore;

namespace WebKit {

void WebPluginListBuilderImpl::addPlugin(const WebString& name, const WebString& description, const WebString& fileName)
{
    PluginInfo* info = new PluginInfo();
    info->name = name;
    info->desc = description;
    info->file = fileName;
    m_results->append(info);
}

void WebPluginListBuilderImpl::addMediaTypeToLastPlugin(const WebString& name, const WebString& description)
{
    MimeClassInfo* info = new MimeClassInfo();
    info->type = name;
    info->desc = description;
    m_results->last()->mimes.append(info);
}

void WebPluginListBuilderImpl::addFileExtensionToLastMediaType(const WebString& extension)
{
    MimeClassInfo* info = m_results->last()->mimes.last();
    if (!info->suffixes.isEmpty())
        info->suffixes.append(',');
    info->suffixes.append(extension);
}

} // namespace WebKit
