
 
#ifndef ProfileGenerator_h
#define ProfileGenerator_h

#include "Profile.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/RefPtr.h>

namespace JSC {

    class ExecState;
    class Profile;
    class ProfileNode;
    class UString;
    struct CallIdentifier;    

    class ProfileGenerator : public RefCounted<ProfileGenerator>  {
    public:
        static PassRefPtr<ProfileGenerator> create(const UString& title, ExecState* originatingExec, unsigned uid);

        // Members
        const UString& title() const;
        PassRefPtr<Profile> profile() const { return m_profile; }
        ExecState* originatingGlobalExec() const { return m_originatingGlobalExec; }
        unsigned profileGroup() const { return m_profileGroup; }

        // Collecting
        void willExecute(const CallIdentifier&);
        void didExecute(const CallIdentifier&);

        // Stopping Profiling
        void stopProfiling();

        typedef void (ProfileGenerator::*ProfileFunction)(const CallIdentifier& callIdentifier);

    private:
        ProfileGenerator(const UString& title, ExecState* originatingExec, unsigned uid);
        void addParentForConsoleStart(ExecState*);

        void removeProfileStart();
        void removeProfileEnd();

        RefPtr<Profile> m_profile;
        ExecState* m_originatingGlobalExec;
        unsigned m_profileGroup;
        RefPtr<ProfileNode> m_head;
        RefPtr<ProfileNode> m_currentNode;
    };

} // namespace JSC

#endif // ProfileGenerator_h
