
 
#ifndef FTPDirectoryDocument_h
#define FTPDirectoryDocument_h

#include "HTMLDocument.h"

namespace WebCore {
    
class DOMImplementation;
    
class FTPDirectoryDocument : public HTMLDocument {
public:
    static PassRefPtr<FTPDirectoryDocument> create(Frame* frame)
    {
        return adoptRef(new FTPDirectoryDocument(frame));
    }

private:
    FTPDirectoryDocument(Frame*);
    virtual Tokenizer* createTokenizer();
};
    
} // namespace WebCore

#endif // FTPDirectoryDocument_h
