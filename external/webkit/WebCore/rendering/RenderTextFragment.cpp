

#include "config.h"
#include "RenderTextFragment.h"

#include "Text.h"

namespace WebCore {

RenderTextFragment::RenderTextFragment(Node* node, StringImpl* str, int startOffset, int length)
    : RenderText(node, str ? str->substring(startOffset, length) : 0)
    , m_start(startOffset)
    , m_end(length)
    , m_firstLetter(0)
{
}

RenderTextFragment::RenderTextFragment(Node* node, StringImpl* str)
    : RenderText(node, str)
    , m_start(0)
    , m_end(str ? str->length() : 0)
    , m_contentString(str)
    , m_firstLetter(0)
{
}

PassRefPtr<StringImpl> RenderTextFragment::originalText() const
{
    Node* e = node();
    RefPtr<StringImpl> result = (e ? static_cast<Text*>(e)->dataImpl() : contentString());
    if (result && (start() > 0 || start() < result->length()))
        result = result->substring(start(), end());
    return result.release();
}

void RenderTextFragment::destroy()
{
    if (m_firstLetter)
        m_firstLetter->destroy();
    RenderText::destroy();
}

void RenderTextFragment::setTextInternal(PassRefPtr<StringImpl> text)
{
    RenderText::setTextInternal(text);
    if (m_firstLetter) {
        ASSERT(!m_contentString);
        m_firstLetter->destroy();
        m_firstLetter = 0;
        m_start = 0;
        m_end = textLength();
    }
}

UChar RenderTextFragment::previousCharacter()
{
    if (start()) {
        Node* e = node();
        StringImpl*  original = (e ? static_cast<Text*>(e)->dataImpl() : contentString());
        if (original)
            return (*original)[start() - 1];
    }

    return RenderText::previousCharacter();
}

} // namespace WebCore
