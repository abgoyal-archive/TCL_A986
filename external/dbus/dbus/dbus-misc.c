
/* -*- mode: C; c-file-style: "gnu" -*- */

#include <config.h>
#include "dbus-misc.h"
#include "dbus-internals.h"
#include "dbus-string.h"


char*
dbus_get_local_machine_id (void)
{
  DBusString uuid;
  char *s;

  s = NULL;
  _dbus_string_init (&uuid);
  if (!_dbus_get_local_machine_uuid_encoded (&uuid) ||
      !_dbus_string_steal_data (&uuid, &s))
    {
      _dbus_string_free (&uuid);
      return FALSE;
    }
  else
    {
      _dbus_string_free (&uuid);
      return s;
    }

}

/** @} */ /* End of public API */

#ifdef DBUS_BUILD_TESTS

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include "dbus-test.h"
#include <stdlib.h>


dbus_bool_t
_dbus_misc_test (void)
{
  
  return TRUE;
}

#endif /* !DOXYGEN_SHOULD_SKIP_THIS */

#endif
