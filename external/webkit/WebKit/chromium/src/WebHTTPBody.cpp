

#include "config.h"
#include "WebHTTPBody.h"

#include "FormData.h"
#include "WebFileInfo.h"

using namespace WebCore;

namespace WebKit {

class WebHTTPBodyPrivate : public FormData {
};

void WebHTTPBody::initialize()
{
    assign(static_cast<WebHTTPBodyPrivate*>(FormData::create().releaseRef()));
}

void WebHTTPBody::reset()
{
    assign(0);
}

void WebHTTPBody::assign(const WebHTTPBody& other)
{
    WebHTTPBodyPrivate* p = const_cast<WebHTTPBodyPrivate*>(other.m_private);
    if (p)
        p->ref();
    assign(p);
}

size_t WebHTTPBody::elementCount() const
{
    ASSERT(!isNull());
    return m_private->elements().size();
}

bool WebHTTPBody::elementAt(size_t index, Element& result) const
{
    ASSERT(!isNull());

    if (index >= m_private->elements().size())
        return false;

    const FormDataElement& element = m_private->elements()[index];

    switch (element.m_type) {
    case FormDataElement::data:
        result.type = Element::TypeData;
        result.data.assign(element.m_data.data(), element.m_data.size());
        result.filePath.reset();
        result.fileStart = 0;
        result.fileLength = 0;
        result.fileInfo.modificationTime = 0.0;
        break;
    case FormDataElement::encodedFile:
        result.type = Element::TypeFile;
        result.data.reset();
        result.filePath = element.m_filename;
        result.fileStart = 0; // FIXME: to be set from FormData.
        result.fileLength = -1; // FIXME: to be set from FormData.
        result.fileInfo.modificationTime = 0.0; // FIXME: to be set from FormData.
        break;
    default:
        ASSERT_NOT_REACHED();
        return false;
    }

    return true;
}

void WebHTTPBody::appendData(const WebData& data)
{
    ensureMutable();
    // FIXME: FormDataElement::m_data should be a SharedBuffer<char>.  Then we
    // could avoid this buffer copy.
    m_private->appendData(data.data(), data.size());
}

void WebHTTPBody::appendFile(const WebString& filePath)
{
    ensureMutable();
    m_private->appendFile(filePath);
}

void WebHTTPBody::appendFile(const WebString& filePath, long long fileStart, long long fileLength, const WebFileInfo& fileInfo)
{
    // FIXME: to be implemented.
}

long long WebHTTPBody::identifier() const
{
    ASSERT(!isNull());
    return m_private->identifier();
}

void WebHTTPBody::setIdentifier(long long identifier)
{
    ensureMutable();
    return m_private->setIdentifier(identifier);
}

WebHTTPBody::WebHTTPBody(const PassRefPtr<FormData>& data)
    : m_private(static_cast<WebHTTPBodyPrivate*>(data.releaseRef()))
{
}

WebHTTPBody& WebHTTPBody::operator=(const PassRefPtr<FormData>& data)
{
    assign(static_cast<WebHTTPBodyPrivate*>(data.releaseRef()));
    return *this;
}

WebHTTPBody::operator PassRefPtr<FormData>() const
{
    return m_private;
}

void WebHTTPBody::assign(WebHTTPBodyPrivate* p)
{
    // p is already ref'd for us by the caller
    if (m_private)
        m_private->deref();
    m_private = p;
}

void WebHTTPBody::ensureMutable()
{
    ASSERT(!isNull());
    if (!m_private->hasOneRef())
        assign(static_cast<WebHTTPBodyPrivate*>(m_private->copy().releaseRef()));
}

} // namespace WebKit
