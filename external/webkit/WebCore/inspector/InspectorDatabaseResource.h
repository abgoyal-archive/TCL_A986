

#ifndef InspectorDatabaseResource_h
#define InspectorDatabaseResource_h

#if ENABLE(DATABASE)

#include "Database.h"
#include "ScriptObject.h"
#include "ScriptState.h"

#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/RefPtr.h>

namespace WebCore {
    class InspectorFrontend;
    
    class InspectorDatabaseResource : public RefCounted<InspectorDatabaseResource> {
    public:
        static PassRefPtr<InspectorDatabaseResource> create(Database* database, const String& domain, const String& name, const String& version)
        {
            return adoptRef(new InspectorDatabaseResource(database, domain, name, version));
        }

        void bind(InspectorFrontend* frontend);
        void unbind();
        Database* database() { return m_database.get(); }
        long id() const { return m_id; }
    private:
        InspectorDatabaseResource(Database*, const String& domain, const String& name, const String& version);
        
        RefPtr<Database> m_database;
        int m_id;
        String m_domain;
        String m_name;
        String m_version;
        bool m_scriptObjectCreated;

        static int s_nextUnusedId;
    };

} // namespace WebCore

#endif // ENABLE(DATABASE)

#endif // InspectorDatabaseResource_h
