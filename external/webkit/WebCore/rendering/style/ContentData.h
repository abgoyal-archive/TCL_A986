

#ifndef ContentData_h
#define ContentData_h

#include "RenderStyleConstants.h"
#include <wtf/Noncopyable.h>
#include <wtf/PassRefPtr.h>

namespace WebCore {

class CounterContent;
class StringImpl;
class StyleImage;

struct ContentData : Noncopyable {
public:
    ContentData()
        : m_type(CONTENT_NONE)
        , m_next(0)
    {
    }

    ~ContentData()
    {
        clear();
    }

    void clear();

    bool isCounter() const { return m_type == CONTENT_COUNTER; }
    bool isImage() const { return m_type == CONTENT_OBJECT; }
    bool isNone() const { return m_type == CONTENT_NONE; }
    bool isText() const { return m_type == CONTENT_TEXT; }

    StyleContentType type() const { return m_type; }

    bool dataEquivalent(const ContentData&) const;

    StyleImage* image() const { return m_content.m_image; }
    void setImage(PassRefPtr<StyleImage> image)
    {
        deleteContent();
        m_type = CONTENT_OBJECT;
        m_content.m_image = image.releaseRef();
    }

    StringImpl* text() const { return m_content.m_text; }
    void setText(PassRefPtr<StringImpl> text)
    {
        deleteContent();
        m_type = CONTENT_TEXT;
        m_content.m_text = text.releaseRef();
    }

    CounterContent* counter() const { return m_content.m_counter; }
    void setCounter(CounterContent* counter)
    {
        deleteContent();
        m_type = CONTENT_COUNTER;
        m_content.m_counter = counter;
    }

    ContentData* next() const { return m_next; }
    void setNext(ContentData* next)
    {
        m_next = next;
    }

private:
    void deleteContent();

    StyleContentType m_type;
    union {
        StyleImage* m_image;
        StringImpl* m_text;
        CounterContent* m_counter;
    } m_content;
    ContentData* m_next;
};

} // namespace WebCore

#endif // ContentData_h
