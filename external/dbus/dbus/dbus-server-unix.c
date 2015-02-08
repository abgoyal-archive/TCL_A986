
/* -*- mode: C; c-file-style: "gnu" -*- */

#include "dbus-internals.h"
#include "dbus-server-unix.h"
#include "dbus-server-socket.h"
#include "dbus-transport-unix.h"
#include "dbus-connection-internal.h"
#include "dbus-sysdeps-unix.h"
#include "dbus-string.h"


DBusServerListenResult
_dbus_server_listen_platform_specific (DBusAddressEntry *entry,
                                       DBusServer      **server_p,
                                       DBusError        *error)
{
  const char *method;

  *server_p = NULL;
  
  method = dbus_address_entry_get_method (entry);

  if (strcmp (method, "unix") == 0)
    {
      const char *path = dbus_address_entry_get_value (entry, "path");
      const char *tmpdir = dbus_address_entry_get_value (entry, "tmpdir");
      const char *abstract = dbus_address_entry_get_value (entry, "abstract");
          
      if (path == NULL && tmpdir == NULL && abstract == NULL)
        {
          _dbus_set_bad_address(error, "unix",
                                "path or tmpdir or abstract",
                                NULL);
          return DBUS_SERVER_LISTEN_BAD_ADDRESS;
        }

      if ((path && tmpdir) ||
          (path && abstract) ||
          (tmpdir && abstract))
        {
          _dbus_set_bad_address(error, NULL, NULL,
                                "cannot specify two of \"path\" and \"tmpdir\" and \"abstract\" at the same time");
          return DBUS_SERVER_LISTEN_BAD_ADDRESS;
        }

      if (tmpdir != NULL)
        {
          DBusString full_path;
          DBusString filename;
              
          if (!_dbus_string_init (&full_path))
            {
              dbus_set_error (error, DBUS_ERROR_NO_MEMORY, NULL);
              return DBUS_SERVER_LISTEN_DID_NOT_CONNECT;
            }
                  
          if (!_dbus_string_init (&filename))
            {
              _dbus_string_free (&full_path);
              dbus_set_error (error, DBUS_ERROR_NO_MEMORY, NULL);
              return DBUS_SERVER_LISTEN_DID_NOT_CONNECT;
            }
              
          if (!_dbus_string_append (&filename,
                                    "dbus-") ||
              !_dbus_generate_random_ascii (&filename, 10) ||
              !_dbus_string_append (&full_path, tmpdir) ||
              !_dbus_concat_dir_and_file (&full_path, &filename))
            {
              _dbus_string_free (&full_path);
              _dbus_string_free (&filename);
              dbus_set_error (error, DBUS_ERROR_NO_MEMORY, NULL);
              return DBUS_SERVER_LISTEN_DID_NOT_CONNECT;
            }
              
          /* Always use abstract namespace if possible with tmpdir */
              
          *server_p =
            _dbus_server_new_for_domain_socket (_dbus_string_get_const_data (&full_path),
#ifdef HAVE_ABSTRACT_SOCKETS
                                                TRUE,
#else
                                                FALSE,
#endif
                                                error);

          _dbus_string_free (&full_path);
          _dbus_string_free (&filename);
        }
      else
        {
          if (path)
            *server_p = _dbus_server_new_for_domain_socket (path, FALSE, error);
          else
            *server_p = _dbus_server_new_for_domain_socket (abstract, TRUE, error);
        }

      if (*server_p != NULL)
        {
          _DBUS_ASSERT_ERROR_IS_CLEAR(error);
          return DBUS_SERVER_LISTEN_OK;
        }
      else
        {
          _DBUS_ASSERT_ERROR_IS_SET(error);
          return DBUS_SERVER_LISTEN_DID_NOT_CONNECT;
        }
    }
  else
    {
      /* If we don't handle the method, we return NULL with the
       * error unset
       */
      _DBUS_ASSERT_ERROR_IS_CLEAR(error);
      return DBUS_SERVER_LISTEN_NOT_HANDLED;
    }
}

DBusServer*
_dbus_server_new_for_domain_socket (const char     *path,
                                    dbus_bool_t     abstract,
                                    DBusError      *error)
{
  DBusServer *server;
  int listen_fd;
  DBusString address;
  char *path_copy;
  DBusString path_str;
  
  _DBUS_ASSERT_ERROR_IS_CLEAR (error);

  if (!_dbus_string_init (&address))
    {
      dbus_set_error (error, DBUS_ERROR_NO_MEMORY, NULL);
      return NULL;
    }

  _dbus_string_init_const (&path_str, path);
  if ((abstract &&
       !_dbus_string_append (&address, "unix:abstract=")) ||
      (!abstract &&
       !_dbus_string_append (&address, "unix:path=")) ||
      !_dbus_address_append_escaped (&address, &path_str))
    {
      dbus_set_error (error, DBUS_ERROR_NO_MEMORY, NULL);
      goto failed_0;
    }

  path_copy = _dbus_strdup (path);
  if (path_copy == NULL)
    {
      dbus_set_error (error, DBUS_ERROR_NO_MEMORY, NULL);
      goto failed_0;
    }
  
  listen_fd = _dbus_listen_unix_socket (path, abstract, error);
  _dbus_fd_set_close_on_exec (listen_fd);
  
  if (listen_fd < 0)
    {
      _DBUS_ASSERT_ERROR_IS_SET (error);
      goto failed_1;
    }
  
  server = _dbus_server_new_for_socket (listen_fd, &address);
  if (server == NULL)
    {
      dbus_set_error (error, DBUS_ERROR_NO_MEMORY, NULL);
      goto failed_2;
    }

  _dbus_server_socket_own_filename(server, path_copy);
  
  _dbus_string_free (&address);
  
  return server;

 failed_2:
  _dbus_close_socket (listen_fd, NULL);
 failed_1:
  dbus_free (path_copy);
 failed_0:
  _dbus_string_free (&address);

  return NULL;
}

/** @} */

