

#ifndef EventSender_h
#define EventSender_h

typedef const struct OpaqueJSContext* JSContextRef;
typedef struct OpaqueJSValue* JSObjectRef;

JSObjectRef makeEventSender(JSContextRef context);
void replaySavedEvents();

#endif
