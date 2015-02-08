
/* -*- mode: C; c-file-style: "gnu" -*- */
#ifndef DBUS_DATASLOT_H
#define DBUS_DATASLOT_H

#include <dbus/dbus-internals.h>

DBUS_BEGIN_DECLS

typedef struct DBusDataSlotAllocator DBusDataSlotAllocator;
typedef struct DBusDataSlotList DBusDataSlotList;

/** Opaque typedef for DBusDataSlot */
typedef struct DBusDataSlot DBusDataSlot;
/** DBusDataSlot is used to store application data on the connection */
struct DBusDataSlot
{
  void *data;                      /**< The application data */
  DBusFreeFunction free_data_func; /**< Free the application data */
};

typedef struct DBusAllocatedSlot DBusAllocatedSlot;

struct DBusAllocatedSlot
{
  dbus_int32_t slot_id;  /**< ID of this slot */
  int          refcount; /**< Number of uses of the slot */
};

struct DBusDataSlotAllocator
{
  DBusAllocatedSlot *allocated_slots; /**< Allocated slots */
  int  n_allocated_slots; /**< number of slots malloc'd */
  int  n_used_slots;      /**< number of slots used */
  DBusMutex **lock_loc;   /**< location of thread lock */
};

struct DBusDataSlotList
{
  DBusDataSlot *slots;   /**< Data slots */
  int           n_slots; /**< Slots we have storage for in data_slots */
};

dbus_bool_t _dbus_data_slot_allocator_init  (DBusDataSlotAllocator  *allocator);
dbus_bool_t _dbus_data_slot_allocator_alloc (DBusDataSlotAllocator  *allocator,
                                             DBusMutex              **mutex_loc,
                                             int                    *slot_id_p);
void        _dbus_data_slot_allocator_free  (DBusDataSlotAllocator  *allocator,
                                             int                    *slot_id_p);
void        _dbus_data_slot_list_init       (DBusDataSlotList       *list);
dbus_bool_t _dbus_data_slot_list_set        (DBusDataSlotAllocator  *allocator,
                                             DBusDataSlotList       *list,
                                             int                     slot,
                                             void                   *data,
                                             DBusFreeFunction        free_data_func,
                                             DBusFreeFunction       *old_free_func,
                                             void                  **old_data);
void*       _dbus_data_slot_list_get        (DBusDataSlotAllocator  *allocator,
                                             DBusDataSlotList       *list,
                                             int                     slot);
void        _dbus_data_slot_list_clear      (DBusDataSlotList       *list);
void        _dbus_data_slot_list_free       (DBusDataSlotList       *list);


DBUS_END_DECLS

#endif /* DBUS_DATASLOT_H */
