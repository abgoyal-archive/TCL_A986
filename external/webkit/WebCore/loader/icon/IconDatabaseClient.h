
 
#ifndef IconDatabaseClient_h
#define IconDatabaseClient_h

#include <wtf/Noncopyable.h>

// All of these client methods will be called from a non-main thread
// Take appropriate measures
 
namespace WebCore {

class String;

class IconDatabaseClient : public Noncopyable {
public:
    virtual ~IconDatabaseClient() { }
    virtual bool performImport() { return true; }
    virtual void dispatchDidRemoveAllIcons() { }
    virtual void dispatchDidAddIconForPageURL(const String& /*pageURL*/) { }
};
 
} // namespace WebCore 

#endif
