

#ifndef SQLiteStatement_h
#define SQLiteStatement_h

#include "SQLiteDatabase.h"

struct sqlite3_stmt;

namespace WebCore {

class SQLValue;

class SQLiteStatement : public Noncopyable {
public:
    SQLiteStatement(SQLiteDatabase&, const String&);
    ~SQLiteStatement();
    
    int prepare();
    int bindBlob(int index, const void* blob, int size);
    int bindText(int index, const String&);
    int bindInt64(int index, int64_t);
    int bindDouble(int index, double);
    int bindNull(int index);
    int bindValue(int index, const SQLValue&);
    unsigned bindParameterCount() const;

    int step();
    int finalize();
    int reset();
    
    int prepareAndStep() { if (int error = prepare()) return error; return step(); }
    
    // prepares, steps, and finalizes the query.
    // returns true if all 3 steps succeed with step() returning SQLITE_DONE
    // returns false otherwise  
    bool executeCommand();
    
    // prepares, steps, and finalizes.  
    // returns true is step() returns SQLITE_ROW
    // returns false otherwise
    bool returnsAtLeastOneResult();

    bool isExpired();

    // Returns -1 on last-step failing.  Otherwise, returns number of rows
    // returned in the last step()
    int columnCount();
    
    String getColumnName(int col);
    SQLValue getColumnValue(int col);
    String getColumnText(int col);
    double getColumnDouble(int col);
    int getColumnInt(int col);
    int64_t getColumnInt64(int col);
    const void* getColumnBlob(int col, int& size);
    void getColumnBlobAsVector(int col, Vector<char>&);

    bool returnTextResults(int col, Vector<String>&);
    bool returnIntResults(int col, Vector<int>&);
    bool returnInt64Results(int col, Vector<int64_t>&);
    bool returnDoubleResults(int col, Vector<double>&);

    SQLiteDatabase* database() { return &m_database; }
    
    const String& query() const { return m_query; }
    
private:
    SQLiteDatabase& m_database;
    String m_query;
    sqlite3_stmt* m_statement;
#ifndef NDEBUG
    bool m_isPrepared;
#endif
};

} // namespace WebCore

#endif // SQLiteStatement_h
