

#ifndef TextDocument_h
#define TextDocument_h

#include "HTMLDocument.h"

namespace WebCore {

class HTMLViewSourceDocument;

class TextDocument : public HTMLDocument {
public:
    static PassRefPtr<TextDocument> create(Frame* frame)
    {
        return adoptRef(new TextDocument(frame));
    }

private:
    TextDocument(Frame*);
    
    virtual Tokenizer* createTokenizer();
};

Tokenizer* createTextTokenizer(HTMLViewSourceDocument*);

}

#endif // TextDocument_h
