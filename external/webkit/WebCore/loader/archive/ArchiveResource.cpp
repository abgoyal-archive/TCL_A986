

#include "config.h"
#include "ArchiveResource.h"

#include "SharedBuffer.h"

namespace WebCore {

PassRefPtr<ArchiveResource> ArchiveResource::create(PassRefPtr<SharedBuffer> data, const KURL& url, const ResourceResponse& response)
{
    return data ? adoptRef(new ArchiveResource(data, url, response)) : 0;
}

PassRefPtr<ArchiveResource> ArchiveResource::create(PassRefPtr<SharedBuffer> data, const KURL& url, const String& mimeType, const String& textEncoding, const String& frameName)
{
    return data ? adoptRef(new ArchiveResource(data, url, mimeType, textEncoding, frameName)) : 0;
}

PassRefPtr<ArchiveResource> ArchiveResource::create(PassRefPtr<SharedBuffer> data, const KURL& url, const String& mimeType, const String& textEncoding, const String& frameName, const ResourceResponse& resourceResponse)
{
    return data ? adoptRef(new ArchiveResource(data, url, mimeType, textEncoding, frameName, resourceResponse)) : 0;
}

ArchiveResource::ArchiveResource(PassRefPtr<SharedBuffer> data, const KURL& url, const ResourceResponse& response)
    : SubstituteResource(url, response, data)
    , m_mimeType(response.mimeType())
    , m_textEncoding(response.textEncodingName())
    , m_shouldIgnoreWhenUnarchiving(false)
{
}

ArchiveResource::ArchiveResource(PassRefPtr<SharedBuffer> data, const KURL& url, const String& mimeType, const String& textEncoding, const String& frameName)
    : SubstituteResource(url, ResourceResponse(url, mimeType, data ? data->size() : 0, textEncoding, String()), data)
    , m_mimeType(mimeType)
    , m_textEncoding(textEncoding)
    , m_frameName(frameName)
    , m_shouldIgnoreWhenUnarchiving(false)
{
}

ArchiveResource::ArchiveResource(PassRefPtr<SharedBuffer> data, const KURL& url, const String& mimeType, const String& textEncoding, const String& frameName, const ResourceResponse& response)
    : SubstituteResource(url, response.isNull() ? ResourceResponse(url, mimeType, data ? data->size() : 0, textEncoding, String()) : response, data)
    , m_mimeType(mimeType)
    , m_textEncoding(textEncoding)
    , m_frameName(frameName)
    , m_shouldIgnoreWhenUnarchiving(false)
{
}

}
