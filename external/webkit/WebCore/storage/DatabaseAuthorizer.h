
#ifndef DatabaseAuthorizer_h
#define DatabaseAuthorizer_h

#include "StringHash.h"
#include <wtf/HashSet.h>
#include <wtf/PassRefPtr.h>
#include <wtf/Threading.h>

namespace WebCore {

class String;

extern const int SQLAuthAllow;
extern const int SQLAuthIgnore;
extern const int SQLAuthDeny;

class DatabaseAuthorizer : public ThreadSafeShared<DatabaseAuthorizer> {
public:
    static PassRefPtr<DatabaseAuthorizer> create() { return adoptRef(new DatabaseAuthorizer); }

    int createTable(const String& tableName);
    int createTempTable(const String& tableName);
    int dropTable(const String& tableName);
    int dropTempTable(const String& tableName);
    int allowAlterTable(const String& databaseName, const String& tableName);

    int createIndex(const String& indexName, const String& tableName);
    int createTempIndex(const String& indexName, const String& tableName);
    int dropIndex(const String& indexName, const String& tableName);
    int dropTempIndex(const String& indexName, const String& tableName);

    int createTrigger(const String& triggerName, const String& tableName);
    int createTempTrigger(const String& triggerName, const String& tableName);
    int dropTrigger(const String& triggerName, const String& tableName);
    int dropTempTrigger(const String& triggerName, const String& tableName);

    int createView(const String& viewName);
    int createTempView(const String& viewName);
    int dropView(const String& viewName);
    int dropTempView(const String& viewName);

    int createVTable(const String& tableName, const String& moduleName);
    int dropVTable(const String& tableName, const String& moduleName);

    int allowDelete(const String& tableName);
    int allowInsert(const String& tableName);
    int allowUpdate(const String& tableName, const String& columnName);
    int allowTransaction();

    int allowSelect() { return SQLAuthAllow; }
    int allowRead(const String& tableName, const String& columnName);

    int allowReindex(const String& indexName);
    int allowAnalyze(const String& tableName);
    int allowFunction(const String& functionName);
    int allowPragma(const String& pragmaName, const String& firstArgument);

    int allowAttach(const String& filename);
    int allowDetach(const String& databaseName);

    void disable();
    void enable();
    void setReadOnly();

    void reset();

    bool lastActionWasInsert() const { return m_lastActionWasInsert; }
    bool lastActionChangedDatabase() const { return m_lastActionChangedDatabase; }

private:
    DatabaseAuthorizer();
    void addWhitelistedFunctions();
    int denyBasedOnTableName(const String&);

    bool m_securityEnabled : 1;
    bool m_lastActionWasInsert : 1;
    bool m_lastActionChangedDatabase : 1;
    bool m_readOnly : 1;

    HashSet<String, CaseFoldingHash> m_whitelistedFunctions;
};

} // namespace WebCore

#endif // DatabaseAuthorizer_h
