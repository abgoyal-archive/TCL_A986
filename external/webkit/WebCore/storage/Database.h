

#ifndef Database_h
#define Database_h

#if ENABLE(DATABASE)
#include "PlatformString.h"
#include "SecurityOrigin.h"
#include "SQLiteDatabase.h"
#include "SQLTransaction.h"
#include "StringHash.h"
#include "Timer.h"
#include "VoidCallback.h"

#include <wtf/Forward.h>
#include <wtf/HashSet.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>
#include <wtf/Deque.h>
#else
#include "PlatformString.h"
#endif

#if ENABLE(DATABASE)
namespace WebCore {

class DatabaseAuthorizer;
class DatabaseThread;
class ScriptExecutionContext;
class SQLResultSet;
class SQLTransactionCallback;
class SQLTransactionClient;
class SQLTransactionCoordinator;
class SQLTransactionErrorCallback;
class SQLValue;

typedef int ExceptionCode;

class Database : public ThreadSafeShared<Database> {
    friend class DatabaseTransactionTask;
    friend class SQLStatement;
    friend class SQLTransaction;
public:
    static void setIsAvailable(bool);
    static bool isAvailable();

    ~Database();

// Direct support for the DOM API
    static PassRefPtr<Database> openDatabase(ScriptExecutionContext* context, const String& name, const String& expectedVersion, const String& displayName, unsigned long estimatedSize, ExceptionCode&);
    String version() const;
    void changeVersion(const String& oldVersion, const String& newVersion,
                       PassRefPtr<SQLTransactionCallback> callback, PassRefPtr<SQLTransactionErrorCallback> errorCallback,
                       PassRefPtr<VoidCallback> successCallback);
    void transaction(PassRefPtr<SQLTransactionCallback> callback, PassRefPtr<SQLTransactionErrorCallback> errorCallback,
                     PassRefPtr<VoidCallback> successCallback, bool readOnly);

// Internal engine support
    static const String& databaseInfoTableName();

    void disableAuthorizer();
    void enableAuthorizer();
    void setAuthorizerReadOnly();

    Vector<String> tableNames();

    ScriptExecutionContext* scriptExecutionContext() const { return m_scriptExecutionContext.get(); }
    SecurityOrigin* securityOrigin() const;
    String stringIdentifier() const;
    String displayName() const;
    unsigned long estimatedSize() const;
    String fileName() const;

    bool getVersionFromDatabase(String&);
    bool setVersionInDatabase(const String&);
    void setExpectedVersion(const String&);
    bool versionMatchesExpected() const;

    void markAsDeletedAndClose();
    bool deleted() const { return m_deleted; }

    void close();
    bool opened() const { return m_opened; }

    void stop();
    bool stopped() const { return m_stopped; }

    unsigned long long databaseSize() const;
    unsigned long long maximumSize() const;

// Called from DatabaseThread, must be prepared to work on the background thread
    void resetAuthorizer();
    void performPolicyChecks();

    bool performOpenAndVerify(ExceptionCode&);

    Vector<String> performGetTableNames();

    SQLTransactionClient* transactionClient() const;
    SQLTransactionCoordinator* transactionCoordinator() const;

private:
    Database(ScriptExecutionContext* context, const String& name,
             const String& expectedVersion, const String& displayName,
             unsigned long estimatedSize);

    bool openAndVerifyVersion(ExceptionCode&);

    void scheduleTransaction();
    void scheduleTransactionCallback(SQLTransaction*);
    void scheduleTransactionStep(SQLTransaction* transaction, bool immediately = false);

    Deque<RefPtr<SQLTransaction> > m_transactionQueue;
    Mutex m_transactionInProgressMutex;
    bool m_transactionInProgress;
    bool m_isTransactionQueueEnabled;

    static void deliverPendingCallback(void*);

    RefPtr<ScriptExecutionContext> m_scriptExecutionContext;
    RefPtr<SecurityOrigin> m_mainThreadSecurityOrigin;
    RefPtr<SecurityOrigin> m_databaseThreadSecurityOrigin;
    String m_name;
    int m_guid;
    String m_expectedVersion;
    String m_displayName;
    unsigned long m_estimatedSize;
    String m_filename;

    bool m_deleted;

    bool m_stopped;

    bool m_opened;

    SQLiteDatabase m_sqliteDatabase;
    RefPtr<DatabaseAuthorizer> m_databaseAuthorizer;

#ifndef NDEBUG
    String databaseDebugName() const { return m_mainThreadSecurityOrigin->toString() + "::" + m_name; }
#endif
};

} // namespace WebCore

#else

namespace WebCore {
class Database : public ThreadSafeShared<Database> {
public:
    static const String& databaseInfoTableName();
};
} // namespace WebCore

#endif // ENABLE(DATABASE)

#endif // Database_h
