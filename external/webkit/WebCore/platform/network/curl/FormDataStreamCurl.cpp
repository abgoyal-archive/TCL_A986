

#include "config.h"
#include "FormDataStreamCurl.h"

#include "CString.h"
#include "FormData.h"
#include "ResourceRequest.h"

namespace WebCore {

FormDataStream::~FormDataStream()
{
    if (m_file)
        fclose(m_file);
}

size_t FormDataStream::read(void* ptr, size_t blockSize, size_t numberOfBlocks)
{
    // Check for overflow.
    if (!numberOfBlocks || blockSize > std::numeric_limits<size_t>::max() / numberOfBlocks)
        return 0;

    Vector<FormDataElement> elements;
    if (m_resourceHandle->request().httpBody())
        elements = m_resourceHandle->request().httpBody()->elements();

    if (m_formDataElementIndex >= elements.size())
        return 0;

    FormDataElement element = elements[m_formDataElementIndex];

    size_t toSend = blockSize * numberOfBlocks;
    size_t sent;

    if (element.m_type == FormDataElement::encodedFile) {
        if (!m_file)
            m_file = fopen(element.m_filename.utf8().data(), "rb");

        if (!m_file) {
            // FIXME: show a user error?
#ifndef NDEBUG
            printf("Failed while trying to open %s for upload\n", element.m_filename.utf8().data());
#endif
            return 0;
        }

        sent = fread(ptr, blockSize, numberOfBlocks, m_file);
        if (!blockSize && ferror(m_file)) {
            // FIXME: show a user error?
#ifndef NDEBUG
            printf("Failed while trying to read %s for upload\n", element.m_filename.utf8().data());
#endif
            return 0;
        }
        if (feof(m_file)) {
            fclose(m_file);
            m_file = 0;
            m_formDataElementIndex++;
        }
    } else {
        size_t elementSize = element.m_data.size() - m_formDataElementDataOffset;
        sent = elementSize > toSend ? toSend : elementSize;
        memcpy(ptr, element.m_data.data() + m_formDataElementDataOffset, sent);
        if (elementSize > sent)
            m_formDataElementDataOffset += sent;
        else {
            m_formDataElementDataOffset = 0;
            m_formDataElementIndex++;
        }
    }

    return sent;
}

bool FormDataStream::hasMoreElements() const
{
    Vector<FormDataElement> elements;
    if (m_resourceHandle->request().httpBody())
        elements = m_resourceHandle->request().httpBody()->elements();

    return m_formDataElementIndex < elements.size();
}

} // namespace WebCore
