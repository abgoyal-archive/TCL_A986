
/* -*- mode: C; c-file-style: "gnu" -*- */

#include "dbus-internals.h"
#include "dbus-connection-internal.h"
#include "dbus-transport-socket.h"
#include "dbus-transport-protected.h"
#include "dbus-watch.h"



typedef struct DBusTransportSocket DBusTransportSocket;

struct DBusTransportSocket
{
  DBusTransport base;                   /**< Parent instance */
  int fd;                               /**< File descriptor. */
  DBusWatch *read_watch;                /**< Watch for readability. */
  DBusWatch *write_watch;               /**< Watch for writability. */

  int max_bytes_read_per_iteration;     /**< To avoid blocking too long. */
  int max_bytes_written_per_iteration;  /**< To avoid blocking too long. */

  int message_bytes_written;            /**< Number of bytes of current
                                         *   outgoing message that have
                                         *   been written.
                                         */
  DBusString encoded_outgoing;          /**< Encoded version of current
                                         *   outgoing message.
                                         */
  DBusString encoded_incoming;          /**< Encoded version of current
                                         *   incoming data.
                                         */
};

static void
free_watches (DBusTransport *transport)
{
  DBusTransportSocket *socket_transport = (DBusTransportSocket*) transport;

  _dbus_verbose ("%s start\n", _DBUS_FUNCTION_NAME);
  
  if (socket_transport->read_watch)
    {
      if (transport->connection)
        _dbus_connection_remove_watch_unlocked (transport->connection,
                                                socket_transport->read_watch);
      _dbus_watch_invalidate (socket_transport->read_watch);
      _dbus_watch_unref (socket_transport->read_watch);
      socket_transport->read_watch = NULL;
    }

  if (socket_transport->write_watch)
    {
      if (transport->connection)
        _dbus_connection_remove_watch_unlocked (transport->connection,
                                                socket_transport->write_watch);
      _dbus_watch_invalidate (socket_transport->write_watch);
      _dbus_watch_unref (socket_transport->write_watch);
      socket_transport->write_watch = NULL;
    }

  _dbus_verbose ("%s end\n", _DBUS_FUNCTION_NAME);
}

static void
socket_finalize (DBusTransport *transport)
{
  DBusTransportSocket *socket_transport = (DBusTransportSocket*) transport;

  _dbus_verbose ("%s\n", _DBUS_FUNCTION_NAME);
  
  free_watches (transport);

  _dbus_string_free (&socket_transport->encoded_outgoing);
  _dbus_string_free (&socket_transport->encoded_incoming);
  
  _dbus_transport_finalize_base (transport);

  _dbus_assert (socket_transport->read_watch == NULL);
  _dbus_assert (socket_transport->write_watch == NULL);
  
  dbus_free (transport);
}

static void
check_write_watch (DBusTransport *transport)
{
  DBusTransportSocket *socket_transport = (DBusTransportSocket*) transport;
  dbus_bool_t needed;

  if (transport->connection == NULL)
    return;

  if (transport->disconnected)
    {
      _dbus_assert (socket_transport->write_watch == NULL);
      return;
    }
  
  _dbus_transport_ref (transport);

  if (_dbus_transport_get_is_authenticated (transport))
    needed = _dbus_connection_has_messages_to_send_unlocked (transport->connection);
  else
    {
      if (transport->send_credentials_pending)
        needed = TRUE;
      else
        {
          DBusAuthState auth_state;
          
          auth_state = _dbus_auth_do_work (transport->auth);
          
          /* If we need memory we install the write watch just in case,
           * if there's no need for it, it will get de-installed
           * next time we try reading.
           */
          if (auth_state == DBUS_AUTH_STATE_HAVE_BYTES_TO_SEND ||
              auth_state == DBUS_AUTH_STATE_WAITING_FOR_MEMORY)
            needed = TRUE;
          else
            needed = FALSE;
        }
    }

  _dbus_verbose ("check_write_watch(): needed = %d on connection %p watch %p fd = %d outgoing messages exist %d\n",
                 needed, transport->connection, socket_transport->write_watch,
                 socket_transport->fd,
                 _dbus_connection_has_messages_to_send_unlocked (transport->connection));

  _dbus_connection_toggle_watch_unlocked (transport->connection,
                                          socket_transport->write_watch,
                                          needed);

  _dbus_transport_unref (transport);
}

static void
check_read_watch (DBusTransport *transport)
{
  DBusTransportSocket *socket_transport = (DBusTransportSocket*) transport;
  dbus_bool_t need_read_watch;

  _dbus_verbose ("%s: fd = %d\n",
                 _DBUS_FUNCTION_NAME, socket_transport->fd);
  
  if (transport->connection == NULL)
    return;

  if (transport->disconnected)
    {
      _dbus_assert (socket_transport->read_watch == NULL);
      return;
    }
  
  _dbus_transport_ref (transport);

  if (_dbus_transport_get_is_authenticated (transport))
    need_read_watch =
      _dbus_counter_get_value (transport->live_messages_size) < transport->max_live_messages_size;
  else
    {
      if (transport->receive_credentials_pending)
        need_read_watch = TRUE;
      else
        {
          /* The reason to disable need_read_watch when not WAITING_FOR_INPUT
           * is to avoid spinning on the file descriptor when we're waiting
           * to write or for some other part of the auth process
           */
          DBusAuthState auth_state;
          
          auth_state = _dbus_auth_do_work (transport->auth);

          /* If we need memory we install the read watch just in case,
           * if there's no need for it, it will get de-installed
           * next time we try reading. If we're authenticated we
           * install it since we normally have it installed while
           * authenticated.
           */
          if (auth_state == DBUS_AUTH_STATE_WAITING_FOR_INPUT ||
              auth_state == DBUS_AUTH_STATE_WAITING_FOR_MEMORY ||
              auth_state == DBUS_AUTH_STATE_AUTHENTICATED)
            need_read_watch = TRUE;
          else
            need_read_watch = FALSE;
        }
    }

  _dbus_verbose ("  setting read watch enabled = %d\n", need_read_watch);
  _dbus_connection_toggle_watch_unlocked (transport->connection,
                                          socket_transport->read_watch,
                                          need_read_watch);

  _dbus_transport_unref (transport);
}

static void
do_io_error (DBusTransport *transport)
{
  _dbus_transport_ref (transport);
  _dbus_transport_disconnect (transport);
  _dbus_transport_unref (transport);
}

/* return value is whether we successfully read any new data. */
static dbus_bool_t
read_data_into_auth (DBusTransport *transport,
                     dbus_bool_t   *oom)
{
  DBusTransportSocket *socket_transport = (DBusTransportSocket*) transport;
  DBusString *buffer;
  int bytes_read;
  
  *oom = FALSE;

  _dbus_auth_get_buffer (transport->auth, &buffer);
  
  bytes_read = _dbus_read_socket (socket_transport->fd,
                                  buffer, socket_transport->max_bytes_read_per_iteration);

  _dbus_auth_return_buffer (transport->auth, buffer,
                            bytes_read > 0 ? bytes_read : 0);

  if (bytes_read > 0)
    {
      _dbus_verbose (" read %d bytes in auth phase\n", bytes_read);

      return TRUE;
    }
  else if (bytes_read < 0)
    {
      /* EINTR already handled for us */

      if (errno == ENOMEM)
        {
          *oom = TRUE;
        }
      else if (errno == EAGAIN ||
               errno == EWOULDBLOCK)
        ; /* do nothing, just return FALSE below */
      else
        {
          _dbus_verbose ("Error reading from remote app: %s\n",
                         _dbus_strerror (errno));
          do_io_error (transport);
        }

      return FALSE;
    }
  else
    {
      _dbus_assert (bytes_read == 0);
      
      _dbus_verbose ("Disconnected from remote app\n");
      do_io_error (transport);

      return FALSE;
    }
}

/* Return value is whether we successfully wrote any bytes */
static dbus_bool_t
write_data_from_auth (DBusTransport *transport)
{
  DBusTransportSocket *socket_transport = (DBusTransportSocket*) transport;
  int bytes_written;
  const DBusString *buffer;

  if (!_dbus_auth_get_bytes_to_send (transport->auth,
                                     &buffer))
    return FALSE;
  
  bytes_written = _dbus_write_socket (socket_transport->fd,
                                      buffer,
                                      0, _dbus_string_get_length (buffer));

  if (bytes_written > 0)
    {
      _dbus_auth_bytes_sent (transport->auth, bytes_written);
      return TRUE;
    }
  else if (bytes_written < 0)
    {
      /* EINTR already handled for us */
      
      if (errno == EAGAIN ||
          errno == EWOULDBLOCK)
        ;
      else
        {
          _dbus_verbose ("Error writing to remote app: %s\n",
                         _dbus_strerror (errno));
          do_io_error (transport);
        }
    }

  return FALSE;
}

static void
exchange_credentials (DBusTransport *transport,
                      dbus_bool_t    do_reading,
                      dbus_bool_t    do_writing)
{
  DBusTransportSocket *socket_transport = (DBusTransportSocket*) transport;
  DBusError error;

  _dbus_verbose ("exchange_credentials: do_reading = %d, do_writing = %d\n",
                  do_reading, do_writing);

  dbus_error_init (&error);
  if (do_writing && transport->send_credentials_pending)
    {
      if (_dbus_send_credentials_unix_socket (socket_transport->fd,
                                              &error))
        {
          transport->send_credentials_pending = FALSE;
        }
      else
        {
          _dbus_verbose ("Failed to write credentials: %s\n", error.message);
          dbus_error_free (&error);
          do_io_error (transport);
        }
    }
  
  if (do_reading && transport->receive_credentials_pending)
    {
      if (_dbus_read_credentials_unix_socket (socket_transport->fd,
                                              &transport->credentials,
                                              &error))
        {
          transport->receive_credentials_pending = FALSE;
        }
      else
        {
          _dbus_verbose ("Failed to read credentials %s\n", error.message);
          dbus_error_free (&error);
          do_io_error (transport);
        }
    }

  if (!(transport->send_credentials_pending ||
        transport->receive_credentials_pending))
    {
      _dbus_auth_set_credentials (transport->auth,
                                  &transport->credentials);
    }
}

static dbus_bool_t
do_authentication (DBusTransport *transport,
                   dbus_bool_t    do_reading,
                   dbus_bool_t    do_writing,
		   dbus_bool_t   *auth_completed)
{
  dbus_bool_t oom;
  dbus_bool_t orig_auth_state;

  oom = FALSE;
  
  orig_auth_state = _dbus_transport_get_is_authenticated (transport);

  /* This is essential to avoid the check_write_watch() at the end,
   * we don't want to add a write watch in do_iteration before
   * we try writing and get EAGAIN
   */
  if (orig_auth_state)
    {
      if (auth_completed)
        *auth_completed = FALSE;
      return TRUE;
    }
  
  _dbus_transport_ref (transport);
  
  while (!_dbus_transport_get_is_authenticated (transport) &&
         _dbus_transport_get_is_connected (transport))
    {      
      exchange_credentials (transport, do_reading, do_writing);
      
      if (transport->send_credentials_pending ||
          transport->receive_credentials_pending)
        {
          _dbus_verbose ("send_credentials_pending = %d receive_credentials_pending = %d\n",
                         transport->send_credentials_pending,
                         transport->receive_credentials_pending);
          goto out;
        }

#define TRANSPORT_SIDE(t) ((t)->is_server ? "server" : "client")
      switch (_dbus_auth_do_work (transport->auth))
        {
        case DBUS_AUTH_STATE_WAITING_FOR_INPUT:
          _dbus_verbose (" %s auth state: waiting for input\n",
                         TRANSPORT_SIDE (transport));
          if (!do_reading || !read_data_into_auth (transport, &oom))
            goto out;
          break;
      
        case DBUS_AUTH_STATE_WAITING_FOR_MEMORY:
          _dbus_verbose (" %s auth state: waiting for memory\n",
                         TRANSPORT_SIDE (transport));
          oom = TRUE;
          goto out;
          break;
      
        case DBUS_AUTH_STATE_HAVE_BYTES_TO_SEND:
          _dbus_verbose (" %s auth state: bytes to send\n",
                         TRANSPORT_SIDE (transport));
          if (!do_writing || !write_data_from_auth (transport))
            goto out;
          break;
      
        case DBUS_AUTH_STATE_NEED_DISCONNECT:
          _dbus_verbose (" %s auth state: need to disconnect\n",
                         TRANSPORT_SIDE (transport));
          do_io_error (transport);
          break;
      
        case DBUS_AUTH_STATE_AUTHENTICATED:
          _dbus_verbose (" %s auth state: authenticated\n",
                         TRANSPORT_SIDE (transport));
          break;
        }
    }

 out:
  if (auth_completed)
    *auth_completed = (orig_auth_state != _dbus_transport_get_is_authenticated (transport));
  
  check_read_watch (transport);
  check_write_watch (transport);
  _dbus_transport_unref (transport);

  if (oom)
    return FALSE;
  else
    return TRUE;
}

/* returns false on oom */
static dbus_bool_t
do_writing (DBusTransport *transport)
{
  int total;
  DBusTransportSocket *socket_transport = (DBusTransportSocket*) transport;
  dbus_bool_t oom;
  
  /* No messages without authentication! */
  if (!_dbus_transport_get_is_authenticated (transport))
    {
      _dbus_verbose ("Not authenticated, not writing anything\n");
      return TRUE;
    }

  if (transport->disconnected)
    {
      _dbus_verbose ("Not connected, not writing anything\n");
      return TRUE;
    }

#if 1
  _dbus_verbose ("do_writing(), have_messages = %d, fd = %d\n",
                 _dbus_connection_has_messages_to_send_unlocked (transport->connection),
                 socket_transport->fd);
#endif
  
  oom = FALSE;
  total = 0;

  while (!transport->disconnected &&
         _dbus_connection_has_messages_to_send_unlocked (transport->connection))
    {
      int bytes_written;
      DBusMessage *message;
      const DBusString *header;
      const DBusString *body;
      int header_len, body_len;
      int total_bytes_to_write;
      
      if (total > socket_transport->max_bytes_written_per_iteration)
        {
          _dbus_verbose ("%d bytes exceeds %d bytes written per iteration, returning\n",
                         total, socket_transport->max_bytes_written_per_iteration);
          goto out;
        }
      
      message = _dbus_connection_get_message_to_send (transport->connection);
      _dbus_assert (message != NULL);
      _dbus_message_lock (message);

#if 0
      _dbus_verbose ("writing message %p\n", message);
#endif
      
      _dbus_message_get_network_data (message,
                                      &header, &body);

      header_len = _dbus_string_get_length (header);
      body_len = _dbus_string_get_length (body);

      if (_dbus_auth_needs_encoding (transport->auth))
        {
          if (_dbus_string_get_length (&socket_transport->encoded_outgoing) == 0)
            {
              if (!_dbus_auth_encode_data (transport->auth,
                                           header, &socket_transport->encoded_outgoing))
                {
                  oom = TRUE;
                  goto out;
                }
              
              if (!_dbus_auth_encode_data (transport->auth,
                                           body, &socket_transport->encoded_outgoing))
                {
                  _dbus_string_set_length (&socket_transport->encoded_outgoing, 0);
                  oom = TRUE;
                  goto out;
                }
            }
          
          total_bytes_to_write = _dbus_string_get_length (&socket_transport->encoded_outgoing);

#if 0
          _dbus_verbose ("encoded message is %d bytes\n",
                         total_bytes_to_write);
#endif
          
          bytes_written =
            _dbus_write_socket (socket_transport->fd,
                                &socket_transport->encoded_outgoing,
                                socket_transport->message_bytes_written,
                                total_bytes_to_write - socket_transport->message_bytes_written);
        }
      else
        {
          total_bytes_to_write = header_len + body_len;

#if 0
          _dbus_verbose ("message is %d bytes\n",
                         total_bytes_to_write);          
#endif
          
          if (socket_transport->message_bytes_written < header_len)
            {
              bytes_written =
                _dbus_write_socket_two (socket_transport->fd,
                                        header,
                                        socket_transport->message_bytes_written,
                                        header_len - socket_transport->message_bytes_written,
                                        body,
                                        0, body_len);
            }
          else
            {
              bytes_written =
                _dbus_write_socket (socket_transport->fd,
                                    body,
                                    (socket_transport->message_bytes_written - header_len),
                                    body_len -
                                    (socket_transport->message_bytes_written - header_len));
            }
        }

      if (bytes_written < 0)
        {
          /* EINTR already handled for us */
          
          if (errno == EAGAIN ||
              errno == EWOULDBLOCK)
            goto out;
          else
            {
              _dbus_verbose ("Error writing to remote app: %s\n",
                             _dbus_strerror (errno));
              do_io_error (transport);
              goto out;
            }
        }
      else
        {
          _dbus_verbose (" wrote %d bytes of %d\n", bytes_written,
                         total_bytes_to_write);
          
          total += bytes_written;
          socket_transport->message_bytes_written += bytes_written;

          _dbus_assert (socket_transport->message_bytes_written <=
                        total_bytes_to_write);
          
          if (socket_transport->message_bytes_written == total_bytes_to_write)
            {
              socket_transport->message_bytes_written = 0;
              _dbus_string_set_length (&socket_transport->encoded_outgoing, 0);

              _dbus_connection_message_sent (transport->connection,
                                             message);
            }
        }
    }

 out:
  if (oom)
    return FALSE;
  else
    return TRUE;
}

/* returns false on out-of-memory */
static dbus_bool_t
do_reading (DBusTransport *transport)
{
  DBusTransportSocket *socket_transport = (DBusTransportSocket*) transport;
  DBusString *buffer;
  int bytes_read;
  int total;
  dbus_bool_t oom;

  _dbus_verbose ("%s: fd = %d\n", _DBUS_FUNCTION_NAME,
                 socket_transport->fd);
  
  /* No messages without authentication! */
  if (!_dbus_transport_get_is_authenticated (transport))
    return TRUE;

  oom = FALSE;
  
  total = 0;

 again:
  
  /* See if we've exceeded max messages and need to disable reading */
  check_read_watch (transport);
  
  if (total > socket_transport->max_bytes_read_per_iteration)
    {
      _dbus_verbose ("%d bytes exceeds %d bytes read per iteration, returning\n",
                     total, socket_transport->max_bytes_read_per_iteration);
      goto out;
    }

  _dbus_assert (socket_transport->read_watch != NULL ||
                transport->disconnected);
  
  if (transport->disconnected)
    goto out;

  if (!dbus_watch_get_enabled (socket_transport->read_watch))
    return TRUE;
  
  if (_dbus_auth_needs_decoding (transport->auth))
    {
      if (_dbus_string_get_length (&socket_transport->encoded_incoming) > 0)
        bytes_read = _dbus_string_get_length (&socket_transport->encoded_incoming);
      else
        bytes_read = _dbus_read_socket (socket_transport->fd,
                                        &socket_transport->encoded_incoming,
                                        socket_transport->max_bytes_read_per_iteration);

      _dbus_assert (_dbus_string_get_length (&socket_transport->encoded_incoming) ==
                    bytes_read);
      
      if (bytes_read > 0)
        {
          int orig_len;
          
          _dbus_message_loader_get_buffer (transport->loader,
                                           &buffer);

          orig_len = _dbus_string_get_length (buffer);
          
          if (!_dbus_auth_decode_data (transport->auth,
                                       &socket_transport->encoded_incoming,
                                       buffer))
            {
              _dbus_verbose ("Out of memory decoding incoming data\n");
              oom = TRUE;
              goto out;
            }

          _dbus_message_loader_return_buffer (transport->loader,
                                              buffer,
                                              _dbus_string_get_length (buffer) - orig_len);

          _dbus_string_set_length (&socket_transport->encoded_incoming, 0);
        }
    }
  else
    {
      _dbus_message_loader_get_buffer (transport->loader,
                                       &buffer);
      
      bytes_read = _dbus_read_socket (socket_transport->fd,
                                      buffer, socket_transport->max_bytes_read_per_iteration);
      
      _dbus_message_loader_return_buffer (transport->loader,
                                          buffer,
                                          bytes_read < 0 ? 0 : bytes_read);
    }
  
  if (bytes_read < 0)
    {
      /* EINTR already handled for us */

      if (errno == ENOMEM)
        {
          _dbus_verbose ("Out of memory in read()/do_reading()\n");
          oom = TRUE;
          goto out;
        }
      else if (errno == EAGAIN ||
               errno == EWOULDBLOCK)
        goto out;
      else
        {
          _dbus_verbose ("Error reading from remote app: %s\n",
                         _dbus_strerror (errno));
          do_io_error (transport);
          goto out;
        }
    }
  else if (bytes_read == 0)
    {
      _dbus_verbose ("Disconnected from remote app\n");
      do_io_error (transport);
      goto out;
    }
  else
    {
      _dbus_verbose (" read %d bytes\n", bytes_read);
      
      total += bytes_read;      

      if (!_dbus_transport_queue_messages (transport))
        {
          oom = TRUE;
          _dbus_verbose (" out of memory when queueing messages we just read in the transport\n");
          goto out;
        }
      
      /* Try reading more data until we get EAGAIN and return, or
       * exceed max bytes per iteration.  If in blocking mode of
       * course we'll block instead of returning.
       */
      goto again;
    }

 out:
  if (oom)
    return FALSE;
  else
    return TRUE;
}

static dbus_bool_t
socket_handle_watch (DBusTransport *transport,
                   DBusWatch     *watch,
                   unsigned int   flags)
{
  DBusTransportSocket *socket_transport = (DBusTransportSocket*) transport;

  _dbus_assert (watch == socket_transport->read_watch ||
                watch == socket_transport->write_watch);
  _dbus_assert (watch != NULL);
  
  /* Disconnect in case of an error.  In case of hangup do not
   * disconnect the transport because data can still be in the buffer
   * and do_reading may need several iteration to read it all (because
   * of its max_bytes_read_per_iteration limit).  The condition where
   * flags == HANGUP (without READABLE) probably never happen in fact.
   */
  if ((flags & DBUS_WATCH_ERROR) ||
      ((flags & DBUS_WATCH_HANGUP) && !(flags & DBUS_WATCH_READABLE)))
    {
      _dbus_verbose ("Hang up or error on watch\n");
      _dbus_transport_disconnect (transport);
      return TRUE;
    }
  
  if (watch == socket_transport->read_watch &&
      (flags & DBUS_WATCH_READABLE))
    {
      dbus_bool_t auth_finished;
#if 1
      _dbus_verbose ("handling read watch %p flags = %x\n",
                     watch, flags);
#endif
      if (!do_authentication (transport, TRUE, FALSE, &auth_finished))
        return FALSE;

      /* We don't want to do a read immediately following
       * a successful authentication.  This is so we
       * have a chance to propagate the authentication
       * state further up.  Specifically, we need to
       * process any pending data from the auth object.
       */
      if (!auth_finished)
	{
	  if (!do_reading (transport))
	    {
	      _dbus_verbose ("no memory to read\n");
	      return FALSE;
	    }
	}
      else
        {
          _dbus_verbose ("Not reading anything since we just completed the authentication\n");
        }
    }
  else if (watch == socket_transport->write_watch &&
           (flags & DBUS_WATCH_WRITABLE))
    {
#if 1
      _dbus_verbose ("handling write watch, have_outgoing_messages = %d\n",
                     _dbus_connection_has_messages_to_send_unlocked (transport->connection));
#endif
      if (!do_authentication (transport, FALSE, TRUE, NULL))
        return FALSE;
      
      if (!do_writing (transport))
        {
          _dbus_verbose ("no memory to write\n");
          return FALSE;
        }

      /* See if we still need the write watch */
      check_write_watch (transport);
    }
#ifdef DBUS_ENABLE_VERBOSE_MODE
  else
    {
      if (watch == socket_transport->read_watch)
        _dbus_verbose ("asked to handle read watch with non-read condition 0x%x\n",
                       flags);
      else if (watch == socket_transport->write_watch)
        _dbus_verbose ("asked to handle write watch with non-write condition 0x%x\n",
                       flags);
      else
        _dbus_verbose ("asked to handle watch %p on fd %d that we don't recognize\n",
                       watch, dbus_watch_get_fd (watch));
    }
#endif /* DBUS_ENABLE_VERBOSE_MODE */

  return TRUE;
}

static void
socket_disconnect (DBusTransport *transport)
{
  DBusTransportSocket *socket_transport = (DBusTransportSocket*) transport;

  _dbus_verbose ("%s\n", _DBUS_FUNCTION_NAME);
  
  free_watches (transport);
  
  _dbus_close_socket (socket_transport->fd, NULL);
  socket_transport->fd = -1;
}

static dbus_bool_t
socket_connection_set (DBusTransport *transport)
{
  DBusTransportSocket *socket_transport = (DBusTransportSocket*) transport;

  _dbus_watch_set_handler (socket_transport->write_watch,
                           _dbus_connection_handle_watch,
                           transport->connection, NULL);

  _dbus_watch_set_handler (socket_transport->read_watch,
                           _dbus_connection_handle_watch,
                           transport->connection, NULL);
  
  if (!_dbus_connection_add_watch_unlocked (transport->connection,
                                            socket_transport->write_watch))
    return FALSE;

  if (!_dbus_connection_add_watch_unlocked (transport->connection,
                                            socket_transport->read_watch))
    {
      _dbus_connection_remove_watch_unlocked (transport->connection,
                                              socket_transport->write_watch);
      return FALSE;
    }

  check_read_watch (transport);
  check_write_watch (transport);

  return TRUE;
}

static  void
socket_do_iteration (DBusTransport *transport,
                   unsigned int   flags,
                   int            timeout_milliseconds)
{
  DBusTransportSocket *socket_transport = (DBusTransportSocket*) transport;
  DBusPollFD poll_fd;
  int poll_res;
  int poll_timeout;

  _dbus_verbose (" iteration flags = %s%s timeout = %d read_watch = %p write_watch = %p fd = %d\n",
                 flags & DBUS_ITERATION_DO_READING ? "read" : "",
                 flags & DBUS_ITERATION_DO_WRITING ? "write" : "",
                 timeout_milliseconds,
                 socket_transport->read_watch,
                 socket_transport->write_watch,
                 socket_transport->fd);
  
  /* the passed in DO_READING/DO_WRITING flags indicate whether to
   * read/write messages, but regardless of those we may need to block
   * for reading/writing to do auth.  But if we do reading for auth,
   * we don't want to read any messages yet if not given DO_READING.
   */

  poll_fd.fd = socket_transport->fd;
  poll_fd.events = 0;
  
  if (_dbus_transport_get_is_authenticated (transport))
    {
      /* This is kind of a hack; if we have stuff to write, then try
       * to avoid the poll. This is probably about a 5% speedup on an
       * echo client/server.
       *
       * If both reading and writing were requested, we want to avoid this
       * since it could have funky effects:
       *   - both ends spinning waiting for the other one to read
       *     data so they can finish writing
       *   - prioritizing all writing ahead of reading
       */
      if ((flags & DBUS_ITERATION_DO_WRITING) &&
          !(flags & (DBUS_ITERATION_DO_READING | DBUS_ITERATION_BLOCK)) &&
          !transport->disconnected &&
          _dbus_connection_has_messages_to_send_unlocked (transport->connection))
        {
          do_writing (transport);

          if (transport->disconnected ||
              !_dbus_connection_has_messages_to_send_unlocked (transport->connection))
            goto out;
        }

      /* If we get here, we decided to do the poll() after all */
      _dbus_assert (socket_transport->read_watch);
      if (flags & DBUS_ITERATION_DO_READING)
	poll_fd.events |= _DBUS_POLLIN;

      _dbus_assert (socket_transport->write_watch);
      if (flags & DBUS_ITERATION_DO_WRITING)
        poll_fd.events |= _DBUS_POLLOUT;
    }
  else
    {
      DBusAuthState auth_state;
      
      auth_state = _dbus_auth_do_work (transport->auth);

      if (transport->receive_credentials_pending ||
          auth_state == DBUS_AUTH_STATE_WAITING_FOR_INPUT)
	poll_fd.events |= _DBUS_POLLIN;

      if (transport->send_credentials_pending ||
          auth_state == DBUS_AUTH_STATE_HAVE_BYTES_TO_SEND)
	poll_fd.events |= _DBUS_POLLOUT;
    }

  if (poll_fd.events)
    {
      if (flags & DBUS_ITERATION_BLOCK)
	poll_timeout = timeout_milliseconds;
      else
	poll_timeout = 0;

      /* For blocking selects we drop the connection lock here
       * to avoid blocking out connection access during a potentially
       * indefinite blocking call. The io path is still protected
       * by the io_path_cond condvar, so we won't reenter this.
       */
      if (flags & DBUS_ITERATION_BLOCK)
        {
          _dbus_verbose ("unlock %s pre poll\n", _DBUS_FUNCTION_NAME);
          _dbus_connection_unlock (transport->connection);
        }
      
    again:
      poll_res = _dbus_poll (&poll_fd, 1, poll_timeout);

      if (poll_res < 0 && errno == EINTR)
	goto again;

      if (flags & DBUS_ITERATION_BLOCK)
        {
          _dbus_verbose ("lock %s post poll\n", _DBUS_FUNCTION_NAME);
          _dbus_connection_lock (transport->connection);
        }
      
      if (poll_res >= 0)
        {
          if (poll_res == 0)
            poll_fd.revents = 0; /* some concern that posix does not guarantee this;
                                  * valgrind flags it as an error. though it probably
                                  * is guaranteed on linux at least.
                                  */
          
          if (poll_fd.revents & _DBUS_POLLERR)
            do_io_error (transport);
          else
            {
              dbus_bool_t need_read = (poll_fd.revents & _DBUS_POLLIN) > 0;
              dbus_bool_t need_write = (poll_fd.revents & _DBUS_POLLOUT) > 0;
	      dbus_bool_t authentication_completed;

              _dbus_verbose ("in iteration, need_read=%d need_write=%d\n",
                             need_read, need_write);
              do_authentication (transport, need_read, need_write,
				 &authentication_completed);

	      /* See comment in socket_handle_watch. */
	      if (authentication_completed)
                goto out;
                                 
              if (need_read && (flags & DBUS_ITERATION_DO_READING))
                do_reading (transport);
              if (need_write && (flags & DBUS_ITERATION_DO_WRITING))
                do_writing (transport);
            }
        }
      else
        {
          _dbus_verbose ("Error from _dbus_poll(): %s\n",
                         _dbus_strerror (errno));
        }
    }


 out:
  /* We need to install the write watch only if we did not
   * successfully write everything. Note we need to be careful that we
   * don't call check_write_watch *before* do_writing, since it's
   * inefficient to add the write watch, and we can avoid it most of
   * the time since we can write immediately.
   * 
   * However, we MUST always call check_write_watch(); DBusConnection code
   * relies on the fact that running an iteration will notice that
   * messages are pending.
   */
  check_write_watch (transport);

  _dbus_verbose (" ... leaving do_iteration()\n");
}

static void
socket_live_messages_changed (DBusTransport *transport)
{
  /* See if we should look for incoming messages again */
  check_read_watch (transport);
}


static dbus_bool_t
socket_get_socket_fd (DBusTransport *transport,
                      int           *fd_p)
{
  DBusTransportSocket *socket_transport = (DBusTransportSocket*) transport;
  
  *fd_p = socket_transport->fd;
  
  return TRUE;
}

static const DBusTransportVTable socket_vtable = {
  socket_finalize,
  socket_handle_watch,
  socket_disconnect,
  socket_connection_set,
  socket_do_iteration,
  socket_live_messages_changed,
  socket_get_socket_fd
};

DBusTransport*
_dbus_transport_new_for_socket (int               fd,
                                const DBusString *server_guid,
                                const DBusString *address)
{
  DBusTransportSocket *socket_transport;
  
  socket_transport = dbus_new0 (DBusTransportSocket, 1);
  if (socket_transport == NULL)
    return NULL;

  if (!_dbus_string_init (&socket_transport->encoded_outgoing))
    goto failed_0;

  if (!_dbus_string_init (&socket_transport->encoded_incoming))
    goto failed_1;
  
  socket_transport->write_watch = _dbus_watch_new (fd,
                                                 DBUS_WATCH_WRITABLE,
                                                 FALSE,
                                                 NULL, NULL, NULL);
  if (socket_transport->write_watch == NULL)
    goto failed_2;
  
  socket_transport->read_watch = _dbus_watch_new (fd,
                                                DBUS_WATCH_READABLE,
                                                FALSE,
                                                NULL, NULL, NULL);
  if (socket_transport->read_watch == NULL)
    goto failed_3;
  
  if (!_dbus_transport_init_base (&socket_transport->base,
                                  &socket_vtable,
                                  server_guid, address))
    goto failed_4;
  
  socket_transport->fd = fd;
  socket_transport->message_bytes_written = 0;
  
  /* These values should probably be tunable or something. */     
  socket_transport->max_bytes_read_per_iteration = 2048;
  socket_transport->max_bytes_written_per_iteration = 2048;
  
  return (DBusTransport*) socket_transport;

 failed_4:
  _dbus_watch_unref (socket_transport->read_watch);
 failed_3:
  _dbus_watch_unref (socket_transport->write_watch);
 failed_2:
  _dbus_string_free (&socket_transport->encoded_incoming);
 failed_1:
  _dbus_string_free (&socket_transport->encoded_outgoing);
 failed_0:
  dbus_free (socket_transport);
  return NULL;
}

DBusTransport*
_dbus_transport_new_for_tcp_socket (const char     *host,
                                    dbus_int32_t    port,
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
  
  if (!_dbus_string_append (&address, "tcp:"))
    goto error;

  if (host != NULL && 
       (!_dbus_string_append (&address, "host=") ||
        !_dbus_string_append (&address, host)))
    goto error;

  if (!_dbus_string_append (&address, ",port=") ||
      !_dbus_string_append_int (&address, port))
    goto error;

  fd = _dbus_connect_tcp_socket (host, port, error);
  if (fd < 0)
    {
      _DBUS_ASSERT_ERROR_IS_SET (error);
      _dbus_string_free (&address);
      return NULL;
    }

  _dbus_fd_set_close_on_exec (fd);
  
  _dbus_verbose ("Successfully connected to tcp socket %s:%d\n",
                 host, port);
  
  transport = _dbus_transport_new_for_socket (fd, NULL, &address);
  if (transport == NULL)
    {
      dbus_set_error (error, DBUS_ERROR_NO_MEMORY, NULL);
      _dbus_close_socket (fd, NULL);
      _dbus_string_free (&address);
      fd = -1;
    }

  _dbus_string_free (&address);
  
  return transport;

error:
  _dbus_string_free (&address);
  dbus_set_error (error, DBUS_ERROR_NO_MEMORY, NULL);
  return NULL;
}

DBusTransportOpenResult
_dbus_transport_open_socket(DBusAddressEntry  *entry,
                            DBusTransport    **transport_p,                            
                            DBusError         *error)
{
  const char *method;
  
  method = dbus_address_entry_get_method (entry);
  _dbus_assert (method != NULL);

  if (strcmp (method, "tcp") == 0)
    {
      const char *host = dbus_address_entry_get_value (entry, "host");
      const char *port = dbus_address_entry_get_value (entry, "port");
      DBusString  str;
      long lport;
      dbus_bool_t sresult;
          
      if (port == NULL)
        {
          _dbus_set_bad_address (error, "tcp", "port", NULL);
          return DBUS_TRANSPORT_OPEN_BAD_ADDRESS;
        }

      _dbus_string_init_const (&str, port);
      sresult = _dbus_string_parse_int (&str, 0, &lport, NULL);
      _dbus_string_free (&str);
          
      if (sresult == FALSE || lport <= 0 || lport > 65535)
        {
          _dbus_set_bad_address (error, NULL, NULL,
                                 "Port is not an integer between 0 and 65535");
          return DBUS_TRANSPORT_OPEN_BAD_ADDRESS;
        }
          
      *transport_p = _dbus_transport_new_for_tcp_socket (host, lport, error);
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

