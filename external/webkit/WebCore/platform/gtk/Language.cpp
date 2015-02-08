

#include "config.h"
#include "Language.h"

#include "CString.h"
#include "GOwnPtr.h"
#include "PlatformString.h"

#include <gtk/gtk.h>
#include <locale.h>
#include <pango/pango.h>

namespace WebCore {

// Using pango_language_get_default() here is not an option, because
// it doesn't support changing the locale in runtime, so it returns
// always the same value.
String defaultLanguage()
{
    char* localeDefault = setlocale(LC_CTYPE, NULL);

    if (!localeDefault)
        return String("c");

    GOwnPtr<gchar> normalizedDefault(g_ascii_strdown(localeDefault, -1));
    char* ptr = strchr(normalizedDefault.get(), '_');

    if (ptr)
        *ptr = '-';

    ptr = strchr(normalizedDefault.get(), '.');

    if (ptr)
        *ptr = '\0';

    return String(normalizedDefault.get());
}

}
