

#ifndef EventSender_h
#define EventSender_h

class DraggingInfo;

typedef long HRESULT;
typedef const struct OpaqueJSContext* JSContextRef;
typedef struct OpaqueJSValue* JSObjectRef;

JSObjectRef makeEventSender(JSContextRef context);
void replaySavedEvents(HRESULT* oleDragAndDropReturnValue = 0);

extern DraggingInfo* draggingInfo;

#endif
