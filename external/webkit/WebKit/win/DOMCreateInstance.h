

#ifndef DOMCreateInstance_h
#define DOMCreateInstance_h

class GEN_DOMObject;

GEN_DOMObject* getDOMWrapper(void* objectHandle);
void setDOMWrapper(void* objectHandle, GEN_DOMObject* wrapper);
void removeDOMWrapper(void* objectHandle);

#endif // DOMCreateInstance_h
