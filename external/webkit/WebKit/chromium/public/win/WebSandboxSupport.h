

#ifndef WebSandboxSupport_h
#define WebSandboxSupport_h

typedef struct HFONT__* HFONT;

namespace WebKit {

// Put methods here that are required due to sandbox restrictions.
class WebSandboxSupport {
public:
    // Sometimes a Win32 API call will fail because a font is not loaded,
    // and due to sandbox restrictions, the current process may be unable
    // to access the filesystem to load the font.  So, this call serves as
    // a failover to ask the embedder to try some other way to load the
    // font (usually by delegating to an empowered process to have it load
    // the font).  Returns true if the font was successfully loaded.
    virtual bool ensureFontLoaded(HFONT) = 0;
};

} // namespace WebKit

#endif
