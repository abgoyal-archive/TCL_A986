

#ifndef NodeList_h
#define NodeList_h

#include "Node.h"

typedef struct {
    unsigned refCount;
    Node* parentNode;
} NodeList;

extern NodeList* NodeList_new(Node* parentNode);
extern unsigned NodeList_length(NodeList*);
extern Node* NodeList_item(NodeList*, unsigned);
extern void NodeList_ref(NodeList*);
extern void NodeList_deref(NodeList*);

#endif /* NodeList_h */
