

#include "config.h"
#include "SQLiteTransaction.h"

#include "SQLiteDatabase.h"

namespace WebCore {

SQLiteTransaction::SQLiteTransaction(SQLiteDatabase& db, bool readOnly)
    : m_db(db)
    , m_inProgress(false)
    , m_readOnly(readOnly)
{
}

SQLiteTransaction::~SQLiteTransaction()
{
    if (m_inProgress)
        rollback();
}

void SQLiteTransaction::begin()
{
    if (!m_inProgress) {
        ASSERT(!m_db.m_transactionInProgress);
        // Call BEGIN IMMEDIATE for a write transaction to acquire
        // a RESERVED lock on the DB file. Otherwise, another write
        // transaction (on another connection) could make changes
        // to the same DB file before this transaction gets to execute
        // any statements. If that happens, this transaction will fail.
        // http://www.sqlite.org/lang_transaction.html
        // http://www.sqlite.org/lockingv3.html#locking
        if (m_readOnly)
            m_inProgress = m_db.executeCommand("BEGIN");
        else
            m_inProgress = m_db.executeCommand("BEGIN IMMEDIATE");
        m_db.m_transactionInProgress = m_inProgress;
    }
}

void SQLiteTransaction::commit()
{
    if (m_inProgress) {
        ASSERT(m_db.m_transactionInProgress);
        m_inProgress = !m_db.executeCommand("COMMIT");
        m_db.m_transactionInProgress = m_inProgress;
    }
}

void SQLiteTransaction::rollback()
{
    // We do not use the 'm_inProgress = m_db.executeCommand("ROLLBACK")' construct here,
    // because m_inProgress should always be set to false after a ROLLBACK, and
    // m_db.executeCommand("ROLLBACK") can sometimes harmlessly fail, thus returning
    // a non-zero/true result (http://www.sqlite.org/lang_transaction.html).
    if (m_inProgress) {
        ASSERT(m_db.m_transactionInProgress);
        m_db.executeCommand("ROLLBACK");
        m_inProgress = false;
        m_db.m_transactionInProgress = false;
    }
}

void SQLiteTransaction::stop()
{
    if (m_inProgress) {
        m_inProgress = false;
        m_db.m_transactionInProgress = false;
    }
}

bool SQLiteTransaction::wasRolledBackBySqlite() const
{
    // According to http://www.sqlite.org/c3ref/get_autocommit.html,
    // the auto-commit flag should be off in the middle of a transaction
    return m_inProgress && m_db.isAutoCommitOn();
}

} // namespace WebCore
