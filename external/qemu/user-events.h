
#ifndef _QEMU_USEREVENTS_H
#define _QEMU_USEREVENTS_H


void  user_event_key(unsigned  key, unsigned  down);
void  user_event_keycode(int  kcode);
void  user_event_keycodes(int *kcodes, int count);
void  user_event_mouse(int dx, int dy, int dz, unsigned buttons_state);
void  user_event_generic(int type, int code, int value);

typedef void QEMUPutGenericEvent(void*  opaque, int  type, int  code, int  value);
void  user_event_register_generic(void* opaque, QEMUPutGenericEvent  callback);

#endif /* _QEMU_USEREVENTS_H */
