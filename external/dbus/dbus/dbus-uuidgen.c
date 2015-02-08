
/* -*- mode: C; c-file-style: "gnu" -*- */
#include "dbus-uuidgen.h"
#include "dbus-internals.h"
#include "dbus-string.h"
#include "dbus-protocol.h"

#ifdef DBUS_WIN
#error "dbus-uuidgen should not be needed on Windows"
#endif


static dbus_bool_t
return_uuid (DBusGUID   *uuid,
             char      **uuid_p,
             DBusError  *error)
{
  if (uuid_p)
    {
      DBusString encoded;
      _dbus_string_init (&encoded);
      if (!_dbus_uuid_encode (uuid, &encoded) ||
          !_dbus_string_steal_data (&encoded, uuid_p))
        {
          _DBUS_SET_OOM (error);
          _dbus_string_free (&encoded);
          return FALSE;
        }
      _dbus_string_free (&encoded);
    }
  return TRUE;
}

dbus_bool_t
dbus_internal_do_not_use_get_uuid (const char *filename,
                                   char      **uuid_p,
                                   dbus_bool_t create_if_not_found,
                                   DBusError  *error)
{
  DBusGUID uuid;
  
  if (filename)
    {
      DBusString filename_str;
      _dbus_string_init_const (&filename_str, filename);
      if (!_dbus_read_uuid_file (&filename_str, &uuid, create_if_not_found, error))
        goto error;
    }
  else
    {
      if (!_dbus_read_local_machine_uuid (&uuid, create_if_not_found, error))
        goto error;
    }

  if (!return_uuid(&uuid, uuid_p, error))
    goto error;

  return TRUE;
  
 error:
  _DBUS_ASSERT_ERROR_IS_SET (error);
  return FALSE;
}

dbus_bool_t
dbus_internal_do_not_use_create_uuid (char      **uuid_p)
{
  DBusGUID uuid;

  _dbus_generate_uuid (&uuid);
  return return_uuid (&uuid, uuid_p, NULL);
}

/** @} */
