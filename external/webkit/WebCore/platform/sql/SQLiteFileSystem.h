

#ifndef SQLiteFileSystem_h
#define SQLiteFileSystem_h

#include "PlatformString.h"
#include <wtf/Threading.h>

struct sqlite3;

namespace WebCore {

class SQLiteDatabase;

// A class that abstracts the file system related operations required
// by the WebKit database code.
class SQLiteFileSystem {
public:
    // Registers a user-defined SQLite VFS.
    static void registerSQLiteVFS();

    // Opens a database file.
    //
    // fileName - The name of the database file.
    // database - The SQLite structure that represents the database stored
    //            in the given file.
    static int openDatabase(const String& fileName, sqlite3** database);

    // Returns the file name for a database.
    //
    // dbDir - The directory where all databases are stored.
    // dbName - The name of the database.
    // originIdentifier - The origin that wants to use this database.
    // db - A database with a number generator used to create unique file names.
    static String getFileNameForNewDatabase(const String& dbDir, const String& dbName,
                                            const String& originIdentifier, SQLiteDatabase* db);

    // Creates an absolute file path given a directory and a file name.
    //
    // path - The directory.
    // fileName - The file name.
    static String appendDatabaseFileNameToPath(const String& path, const String& fileName);

    // Makes sure the given directory exists, by creating all missing directories
    // on the given path.
    //
    // path - The directory.
    static bool ensureDatabaseDirectoryExists(const String& path);

    // If 'checkPathOnly' is false, then this method only checks if the given file exists.
    // If 'checkPathOnly' is true, then this method makes sure all directories on the
    // given path exist by creating the missing ones, and does not check if the file
    // itself exists.
    //
    // Sometimes we expect a DB file to exist; other times, we're OK with creating a new
    // DB file, but we want to make sure that the directory in which we want to put the
    // new DB file exists. This method covers both cases.
    //
    // fileName - The file name.
    // checkPathOnly - If true, we only make sure that the given directory exists.
    //                 If false, we only check if the file exists.
    static bool ensureDatabaseFileExists(const String& fileName, bool checkPathOnly);

    // Deletes an empty database directory.
    //
    // path - The directory.
    static bool deleteEmptyDatabaseDirectory(const String& path);

    // Deletes a database file.
    //
    // fileName - The file name.
    static bool deleteDatabaseFile(const String& fileName);

    // Returns the size of the database file.
    //
    // fileName - The file name.
    static long long getDatabaseFileSize(const String& fileName);

private:
    // do not instantiate this class
    SQLiteFileSystem();
}; // class SQLiteFileSystem

} // namespace WebCore

#endif
