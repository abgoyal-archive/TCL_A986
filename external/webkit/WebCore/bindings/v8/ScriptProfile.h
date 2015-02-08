

#ifndef ScriptProfile_h
#define ScriptProfile_h

#include "PlatformString.h"

namespace WebCore {

class ScriptProfile : public RefCounted<ScriptProfile> {
public:
    static PassRefPtr<ScriptProfile> create(const String& title, unsigned uid)
    {
      return adoptRef(new ScriptProfile(title, uid));
    }
    virtual ~ScriptProfile() {}

    String title() const { return m_title; }
    unsigned int uid() const { return m_uid; }

protected:
    ScriptProfile(const String& title, unsigned uid)
        : m_title(title)
        , m_uid(uid)
    {}

private:
    String m_title;
    unsigned int m_uid;
};

} // namespace WebCore

#endif // ScriptProfile_h
