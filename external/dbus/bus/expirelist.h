
/* -*- mode: C; c-file-style: "gnu" -*- */

#ifndef BUS_EXPIRE_LIST_H
#define BUS_EXPIRE_LIST_H

#include <dbus/dbus.h>
#include <dbus/dbus-list.h>
#include <dbus/dbus-mainloop.h>

typedef struct BusExpireList BusExpireList;
typedef struct BusExpireItem BusExpireItem;

typedef dbus_bool_t (* BusExpireFunc) (BusExpireList *list,
                                       DBusList      *link,
                                       void          *data);

struct BusExpireList
{
  DBusList      *items; /**< List of BusExpireItem */
  DBusTimeout   *timeout;
  DBusLoop      *loop;
  BusExpireFunc  expire_func;
  void          *data;
  int            expire_after; /**< Expire after milliseconds (thousandths) */
};

/* embed this in a child expire item struct */
struct BusExpireItem
{
  long added_tv_sec;  /**< Time we were added (seconds component) */
  long added_tv_usec; /**< Time we were added (microsec component) */
};

BusExpireList* bus_expire_list_new    (DBusLoop      *loop,
                                       int            expire_after,
                                       BusExpireFunc  expire_func,
                                       void          *data);
void           bus_expire_list_free   (BusExpireList *list);

#define ELAPSED_MILLISECONDS_SINCE(orig_tv_sec, orig_tv_usec,   \
                                   now_tv_sec, now_tv_usec)     \
 (((double) (now_tv_sec) - (double) (orig_tv_sec)) * 1000.0 +   \
 ((double) (now_tv_usec) - (double) (orig_tv_usec)) / 1000.0)

void bus_expire_timeout_set_interval (DBusTimeout *timeout,
                                      int          next_interval);

#endif /* BUS_EXPIRE_LIST_H */
