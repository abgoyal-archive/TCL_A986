

#ifndef HTMLViewSourceDocument_h
#define HTMLViewSourceDocument_h

#include "HTMLDocument.h"

namespace WebCore {

class DoctypeToken;
class HTMLTableCellElement;
class HTMLTableSectionElement;

struct Token;

class HTMLViewSourceDocument : public HTMLDocument {
public:
    static PassRefPtr<HTMLViewSourceDocument> create(Frame* frame, const String& mimeType)
    {
        return adoptRef(new HTMLViewSourceDocument(frame, mimeType));
    }

    void addViewSourceToken(Token*); // Used by the HTML tokenizer.
    void addViewSourceText(const String&); // Used by the plaintext tokenizer.
    void addViewSourceDoctypeToken(DoctypeToken*);

private:
    HTMLViewSourceDocument(Frame*, const String& mimeType);

    // Returns HTMLTokenizer or TextTokenizer based on m_type.
    virtual Tokenizer* createTokenizer();

    void createContainingTable();
    PassRefPtr<Element> addSpanWithClassName(const String&);
    void addLine(const String& className);
    void addText(const String& text, const String& className);
    PassRefPtr<Element> addLink(const String& url, bool isAnchor);

    String m_type;
    RefPtr<Element> m_current;
    RefPtr<HTMLTableSectionElement> m_tbody;
    RefPtr<HTMLTableCellElement> m_td;
};

}

#endif // HTMLViewSourceDocument_h
