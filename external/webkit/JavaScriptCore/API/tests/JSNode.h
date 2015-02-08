

#ifndef JSNode_h
#define JSNode_h

#include "JSBase.h"
#include "Node.h"
#include <stddef.h>

extern JSObjectRef JSNode_new(JSContextRef context, Node* node);
extern JSClassRef JSNode_class(JSContextRef context);
extern JSObjectRef JSNode_construct(JSContextRef context, JSObjectRef object, size_t argumentCount, const JSValueRef arguments[], JSValueRef* exception);

#endif /* JSNode_h */
