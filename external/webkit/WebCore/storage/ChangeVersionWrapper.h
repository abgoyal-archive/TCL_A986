
#ifndef ChangeVersionWrapper_h
#define ChangeVersionWrapper_h

#if ENABLE(DATABASE)

#include "PlatformString.h"
#include "SQLTransaction.h"

namespace WebCore {

class ChangeVersionWrapper : public SQLTransactionWrapper {
public:
    static PassRefPtr<ChangeVersionWrapper> create(const String& oldVersion, const String& newVersion) { return adoptRef(new ChangeVersionWrapper(oldVersion, newVersion)); }

    virtual bool performPreflight(SQLTransaction*);
    virtual bool performPostflight(SQLTransaction*);

    virtual SQLError* sqlError() const { return m_sqlError.get(); }

private:
    ChangeVersionWrapper(const String& oldVersion, const String& newVersion);

    String m_oldVersion;
    String m_newVersion;
    RefPtr<SQLError> m_sqlError;
};

} // namespace WebCore

#endif

#endif // ChangeVersionWrapper_h
