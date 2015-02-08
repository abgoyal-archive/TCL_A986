
 
#ifndef Profile_h
#define Profile_h

#include "ProfileNode.h"
#include <runtime/UString.h>
#include <wtf/RefCounted.h>
#include <wtf/RefPtr.h>

namespace JSC {

    class Profile : public RefCounted<Profile> {
    public:
        static PassRefPtr<Profile> create(const UString& title, unsigned uid);
        virtual ~Profile();

        const UString& title() const { return m_title; }
        ProfileNode* head() const { return m_head.get(); }
        void setHead(PassRefPtr<ProfileNode> head) { m_head = head; }
        double totalTime() const { return m_head->totalTime(); }
        unsigned int uid() const { return m_uid; }

        void forEach(void (ProfileNode::*)());

        void focus(const ProfileNode*);
        void exclude(const ProfileNode*);
        void restoreAll();

#ifndef NDEBUG
        void debugPrintData() const;
        void debugPrintDataSampleStyle() const;
#endif

    protected:
        Profile(const UString& title, unsigned uid);

    private:
        void removeProfileStart();
        void removeProfileEnd();
 
        UString m_title;
        RefPtr<ProfileNode> m_head;
        unsigned int m_uid;
    };

} // namespace JSC

#endif // Profile_h
