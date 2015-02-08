

#ifndef SQLiteTransaction_h
#define SQLiteTransaction_h

#include <wtf/Noncopyable.h>

namespace WebCore {

class SQLiteDatabase;

class SQLiteTransaction : public Noncopyable
{
public:
    SQLiteTransaction(SQLiteDatabase& db, bool readOnly = false);
    ~SQLiteTransaction();
    
    void begin();
    void commit();
    void rollback();
    void stop();
    
    bool inProgress() const { return m_inProgress; }
    bool wasRolledBackBySqlite() const;
private:
    SQLiteDatabase& m_db;
    bool m_inProgress;
    bool m_readOnly;
};

} // namespace WebCore

#endif // SQLiteTransation_H
