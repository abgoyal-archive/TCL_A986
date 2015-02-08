

#ifndef PluginDocument_h
#define PluginDocument_h

#include "HTMLDocument.h"

namespace WebCore {
    
class PluginDocument : public HTMLDocument {
public:
    static PassRefPtr<PluginDocument> create(Frame* frame)
    {
        return adoptRef(new PluginDocument(frame));
    }

private:
    PluginDocument(Frame*);

    virtual bool isPluginDocument() const { return true; }
    virtual Tokenizer* createTokenizer();
};
    
}

#endif // PluginDocument_h
