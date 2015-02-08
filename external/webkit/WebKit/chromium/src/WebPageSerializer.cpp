

#include "config.h"
#include "WebPageSerializer.h"

#include "KURL.h"
#include "PlatformString.h"

#include "WebFrame.h"
#include "WebPageSerializerClient.h"
#include "WebPageSerializerImpl.h"
#include "WebString.h"
#include "WebURL.h"
#include "WebVector.h"

using namespace WebCore;

namespace WebKit {

bool WebPageSerializer::serialize(WebFrame* frame,
                                  bool recursive,
                                  WebPageSerializerClient* client,
                                  const WebVector<WebURL>& links,
                                  const WebVector<WebString>& localPaths,
                                  const WebString& localDirectoryName)
{
    WebPageSerializerImpl serializerImpl(
        frame, recursive, client, links, localPaths, localDirectoryName);
    return serializerImpl.serialize();
}

WebString WebPageSerializer::generateMetaCharsetDeclaration(const WebString& charset)
{
    return String::format("<META http-equiv=\"Content-Type\" content=\"text/html; charset=%s\">",
                          charset.utf8().data());
}

WebString WebPageSerializer::generateMarkOfTheWebDeclaration(const WebURL& url)
{
    return String::format("\n<!-- saved from url=(%04d)%s -->\n",
                          static_cast<int>(url.spec().length()),
                          url.spec().data());
}

WebString WebPageSerializer::generateBaseTagDeclaration(const WebString& baseTarget)
{
    String targetDeclaration;
    if (!baseTarget.isEmpty())
        targetDeclaration = String::format(" target=\"%s\"", baseTarget.utf8().data());
    return String::format("<BASE href=\".\"%s>", targetDeclaration.utf8().data());
}

}  // namespace WebKit
