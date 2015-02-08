

#include "config.h"
#include "InspectorDatabaseResource.h"

#if ENABLE(DATABASE) && ENABLE(INSPECTOR)
#include "Database.h"
#include "Document.h"
#include "Frame.h"
#include "InspectorFrontend.h"
#include "ScriptObject.h"

namespace WebCore {

int InspectorDatabaseResource::s_nextUnusedId = 1;

InspectorDatabaseResource::InspectorDatabaseResource(Database* database, const String& domain, const String& name, const String& version)
    : m_database(database)
    , m_id(s_nextUnusedId++)
    , m_domain(domain)
    , m_name(name)
    , m_version(version)
    , m_scriptObjectCreated(false)
{
}

void InspectorDatabaseResource::bind(InspectorFrontend* frontend)
{
    if (m_scriptObjectCreated)
        return;

    ScriptObject jsonObject = frontend->newScriptObject();
    jsonObject.set("id", m_id);
    jsonObject.set("domain", m_domain);
    jsonObject.set("name", m_name);
    jsonObject.set("version", m_version);
    if (frontend->addDatabase(jsonObject))
        m_scriptObjectCreated = true;
}

void InspectorDatabaseResource::unbind()
{
    m_scriptObjectCreated = false;
}

} // namespace WebCore

#endif // ENABLE(DATABASE) && ENABLE(INSPECTOR)
