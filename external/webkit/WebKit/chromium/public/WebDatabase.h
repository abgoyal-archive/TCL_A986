

#ifndef WebDatabase_h
#define WebDatabase_h

#include "WebCommon.h"
#include "WebSecurityOrigin.h"

#if WEBKIT_IMPLEMENTATION
namespace WebCore { class Database; }
namespace WTF { template <typename T> class PassRefPtr; }
#endif

namespace WebKit {

class WebDatabaseObserver;
class WebDatabasePrivate;
class WebString;

class WebDatabase {
public:
    WebDatabase() : m_private(0) { }
    WebDatabase(const WebDatabase& d) : m_private(0) { assign(d); }
    ~WebDatabase() { reset(); }

    WebDatabase& operator=(const WebDatabase& d)
    {
        assign(d);
        return *this;
    }

    WEBKIT_API void reset();
    WEBKIT_API void assign(const WebDatabase&);
    bool isNull() const { return !m_private; }

    WEBKIT_API WebString name() const;
    WEBKIT_API WebString displayName() const;
    WEBKIT_API unsigned long estimatedSize() const;
    WEBKIT_API WebSecurityOrigin securityOrigin() const;

    WEBKIT_API static void setObserver(WebDatabaseObserver*);
    WEBKIT_API static WebDatabaseObserver* observer();

    WEBKIT_API static void updateDatabaseSize(
        const WebString& originIdentifier, const WebString& databaseName,
        unsigned long long databaseSize, unsigned long long spaceAvailable);
    WEBKIT_API static void closeDatabaseImmediately(
        const WebString& originIdentifier, const WebString& databaseName);

#if WEBKIT_IMPLEMENTATION
    WebDatabase(const WTF::PassRefPtr<WebCore::Database>&);
    WebDatabase& operator=(const WTF::PassRefPtr<WebCore::Database>&);
    operator WTF::PassRefPtr<WebCore::Database>() const;
#endif

private:
    void assign(WebDatabasePrivate*);

    WebDatabasePrivate* m_private;
};

} // namespace WebKit

#endif
