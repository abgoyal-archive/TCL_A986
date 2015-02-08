
/* -*- mode: C; c-file-style: "gnu" -*- */

#ifndef DBUS_MAINLOOP_H
#define DBUS_MAINLOOP_H

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <dbus/dbus.h>

typedef struct DBusLoop DBusLoop;

typedef dbus_bool_t (* DBusWatchFunction)   (DBusWatch     *watch,
                                             unsigned int   condition,
                                             void          *data);
typedef void        (* DBusTimeoutFunction) (DBusTimeout   *timeout,
                                             void          *data);

DBusLoop*   _dbus_loop_new            (void);
DBusLoop*   _dbus_loop_ref            (DBusLoop            *loop);
void        _dbus_loop_unref          (DBusLoop            *loop);
dbus_bool_t _dbus_loop_add_watch      (DBusLoop            *loop,
                                       DBusWatch           *watch,
                                       DBusWatchFunction    function,
                                       void                *data,
                                       DBusFreeFunction     free_data_func);
void        _dbus_loop_remove_watch   (DBusLoop            *loop,
                                       DBusWatch           *watch,
                                       DBusWatchFunction    function,
                                       void                *data);
dbus_bool_t _dbus_loop_add_timeout    (DBusLoop            *loop,
                                       DBusTimeout         *timeout,
                                       DBusTimeoutFunction  function,
                                       void                *data,
                                       DBusFreeFunction     free_data_func);
void        _dbus_loop_remove_timeout (DBusLoop            *loop,
                                       DBusTimeout         *timeout,
                                       DBusTimeoutFunction  function,
                                       void                *data);

dbus_bool_t _dbus_loop_queue_dispatch (DBusLoop            *loop,
                                       DBusConnection      *connection);

void        _dbus_loop_run            (DBusLoop            *loop);
void        _dbus_loop_quit           (DBusLoop            *loop);
dbus_bool_t _dbus_loop_iterate        (DBusLoop            *loop,
                                       dbus_bool_t          block);
dbus_bool_t _dbus_loop_dispatch       (DBusLoop            *loop);

int  _dbus_get_oom_wait    (void);
void _dbus_wait_for_memory (void);

#endif /* !DOXYGEN_SHOULD_SKIP_THIS */

#endif /* DBUS_MAINLOOP_H */

