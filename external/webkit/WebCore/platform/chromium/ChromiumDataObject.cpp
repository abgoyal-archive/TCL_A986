

#include "config.h"
#include "ChromiumDataObject.h"

namespace WebCore {

void ChromiumDataObject::clear()
{
    url = KURL();
    urlTitle = "";
    downloadURL = KURL();
    downloadMetadata = "";
    fileExtension = "";
    filenames.clear();
    plainText = "";
    textHtml = "";
    htmlBaseUrl = KURL();
    fileContentFilename = "";
    if (fileContent)
        fileContent->clear();
}

bool ChromiumDataObject::hasData() const
{
    return !url.isEmpty()
        || !downloadURL.isEmpty()
        || !downloadMetadata.isEmpty()
        || !fileExtension.isEmpty()
        || !filenames.isEmpty()
        || !plainText.isEmpty()
        || !textHtml.isEmpty()
        || fileContent;
}

ChromiumDataObject::ChromiumDataObject(const ChromiumDataObject& other)
    : url(other.url)
    , urlTitle(other.urlTitle)
    , downloadURL(other.downloadURL)
    , downloadMetadata(other.downloadMetadata)
    , fileExtension(other.fileExtension)
    , filenames(other.filenames)
    , plainText(other.plainText)
    , textHtml(other.textHtml)
    , htmlBaseUrl(other.htmlBaseUrl)
    , fileContentFilename(other.fileContentFilename)
{
    if (other.fileContent.get())
        fileContent = other.fileContent->copy();
}

} // namespace WebCore
