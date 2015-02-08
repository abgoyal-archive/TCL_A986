
/* -*- mode: C; c-file-style: "gnu" -*- */

#include "dbus-internals.h"
#include "dbus-connection-internal.h"
#include "dbus-transport-unix.h"
#include "dbus-transport-socket.h"
#include "dbus-transport-protected.h"
#include "dbus-watch.h"
#include "dbus-sysdeps-unix.h"


DBusTransport*
_dbus_transport_new_for_domain_socket (const char     *path,
                                       dbus_bool_t     abstract,
                                       DBusError      *error)
{
  int fd;
  DBusTransport *transport;
  DBusString address;
  
  _DBUS_ASSERT_ERROR_IS_CLEAR (error);

  if (!_dbus_string_init (&address))
    {
      dbus_set_error (error, DBUS_ERROR_NO_MEMORY, NULL);
      return NULL;
    }

  fd = -1;

  if ((abstract &&
       !_dbus_string_append (&address, "unix:abstract=")) ||
      (!abstract &&
       !_dbus_string_append (&address, "unix:path=")) ||
      !_dbus_string_append (&address, path))
    {
      dbus_set_error (error, DBUS_ERROR_NO_MEMORY, NULL);
      goto failed_0;
    }
  
  fd = _dbus_connect_unix_socket (path, abstract, error);
  if (fd < 0)
    {
      _DBUS_ASSERT_ERROR_IS_SET (error);
      goto failed_0;
    }

  _dbus_fd_set_close_on_exec (fd);
  
  _dbus_verbose ("Successfully connected to unix socket %s\n",
                 path);

  transport = _dbus_transport_new_for_socket (fd, NULL, &address);
  if (transport == NULL)
    {
      dbus_set_error (error, DBUS_ERROR_NO_MEMORY, NULL);
      goto failed_1;
    }
  
  _dbus_string_free (&address);
  
  return transport;

 failed_1:
  _dbus_close_socket (fd, NULL);
 failed_0:
  _dbus_string_free (&address);
  return NULL;
}

DBusTransportOpenResult
_dbus_transport_open_platform_specific (DBusAddressEntry  *entry,
                                        DBusTransport    **transport_p,
                                        DBusError         *error)
{
  const char *method;
  
  method = dbus_address_entry_get_method (entry);
  _dbus_assert (method != NULL);

  if (strcmp (method, "unix") == 0)
    {
      const char *path = dbus_address_entry_get_value (entry, "path");
      const char *tmpdir = dbus_address_entry_get_value (entry, "tmpdir");
      const char *abstract = dbus_address_entry_get_value (entry, "abstract");
          
      if (tmpdir != NULL)
        {
          _dbus_set_bad_address (error, NULL, NULL,
                                 "cannot use the \"tmpdir\" option for an address to connect to, only in an address to listen on");
          return DBUS_TRANSPORT_OPEN_BAD_ADDRESS;
        }
          
      if (path == NULL && abstract == NULL)
        {
          _dbus_set_bad_address (error, "unix",
                                 "path or abstract",
                                 NULL);
          return DBUS_TRANSPORT_OPEN_BAD_ADDRESS;
        }

      if (path != NULL && abstract != NULL)
        {
          _dbus_set_bad_address (error, NULL, NULL,
                                 "can't specify both \"path\" and \"abstract\" options in an address");
          return DBUS_TRANSPORT_OPEN_BAD_ADDRESS;
        }

      if (path)
        *transport_p = _dbus_transport_new_for_domain_socket (path, FALSE,
                                                           error);
      else
        *transport_p = _dbus_transport_new_for_domain_socket (abstract, TRUE,
                                                           error);
      if (*transport_p == NULL)
        {
          _DBUS_ASSERT_ERROR_IS_SET (error);
          return DBUS_TRANSPORT_OPEN_DID_NOT_CONNECT;
        }
      else
        {
          _DBUS_ASSERT_ERROR_IS_CLEAR (error);
          return DBUS_TRANSPORT_OPEN_OK;
        }      
    }
  else
    {
      _DBUS_ASSERT_ERROR_IS_CLEAR (error);
      return DBUS_TRANSPORT_OPEN_NOT_HANDLED;
    }
}

/** @} */
