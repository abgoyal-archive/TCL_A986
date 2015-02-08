

#ifndef Text_h
#define Text_h

#include "CharacterData.h"

namespace WebCore {
    
class Text : public CharacterData {
public:
    static const unsigned defaultLengthLimit = 1 << 16;

    static PassRefPtr<Text> create(Document*, const String&);
    static PassRefPtr<Text> createWithLengthLimit(Document*, const String&, unsigned& charsLeft, unsigned lengthLimit = defaultLengthLimit);

    PassRefPtr<Text> splitText(unsigned offset, ExceptionCode&);

    // DOM Level 3: http://www.w3.org/TR/DOM-Level-3-Core/core.html#ID-1312295772

    String wholeText() const;
    PassRefPtr<Text> replaceWholeText(const String&, ExceptionCode&);

    virtual void attach();

protected:
    Text(Document*, const String&);

private:
    virtual String nodeName() const;
    virtual NodeType nodeType() const;
    virtual PassRefPtr<Node> cloneNode(bool deep);
    virtual bool rendererIsNeeded(RenderStyle*);
    virtual RenderObject* createRenderer(RenderArena*, RenderStyle*);
    virtual void recalcStyle(StyleChange = NoChange);
    virtual bool childTypeAllowed(NodeType);

    virtual PassRefPtr<Text> virtualCreate(const String&);

#ifndef NDEBUG
    virtual void formatForDebugger(char* buffer, unsigned length) const;
#endif
};

} // namespace WebCore

#endif // Text_h
