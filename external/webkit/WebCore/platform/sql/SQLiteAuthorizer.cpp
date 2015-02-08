
#include "config.h"
#include "DatabaseAuthorizer.h"

#include <sqlite3.h>

namespace WebCore {

const int SQLAuthAllow = SQLITE_OK;
const int SQLAuthIgnore = SQLITE_IGNORE;
const int SQLAuthDeny = SQLITE_DENY;

} // namespace WebCore
