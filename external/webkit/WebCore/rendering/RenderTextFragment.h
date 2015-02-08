

#ifndef RenderTextFragment_h
#define RenderTextFragment_h

#include "RenderText.h"

namespace WebCore {

// Used to represent a text substring of an element, e.g., for text runs that are split because of
// first letter and that must therefore have different styles (and positions in the render tree).
// We cache offsets so that text transformations can be applied in such a way that we can recover
// the original unaltered string from our corresponding DOM node.
class RenderTextFragment : public RenderText {
public:
    RenderTextFragment(Node*, StringImpl*, int startOffset, int length);
    RenderTextFragment(Node*, StringImpl*);

    virtual bool isTextFragment() const { return true; }

    virtual void destroy();

    unsigned start() const { return m_start; }
    unsigned end() const { return m_end; }

    RenderObject* firstLetter() const { return m_firstLetter; }
    void setFirstLetter(RenderObject* firstLetter) { m_firstLetter = firstLetter; }

    StringImpl* contentString() const { return m_contentString.get(); }
    virtual PassRefPtr<StringImpl> originalText() const;

private:
    virtual void setTextInternal(PassRefPtr<StringImpl>);
    virtual UChar previousCharacter();

    unsigned m_start;
    unsigned m_end;
    RefPtr<StringImpl> m_contentString;
    RenderObject* m_firstLetter;
};

} // namespace WebCore

#endif // RenderTextFragment_h
