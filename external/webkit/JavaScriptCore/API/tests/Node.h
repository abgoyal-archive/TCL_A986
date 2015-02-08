

#ifndef Node_h
#define Node_h

typedef struct __Node Node;
typedef struct __NodeLink NodeLink;

struct __NodeLink {
    Node* node;
    NodeLink* prev;
};

struct __Node {
    unsigned refCount;
    const char* nodeType;
    NodeLink* childNodesTail;
};

extern Node* Node_new(void);
extern void Node_ref(Node* node);
extern void Node_deref(Node* node);
extern void Node_appendChild(Node* node, Node* child);
extern void Node_removeChild(Node* node, Node* child);
extern void Node_replaceChild(Node* node, Node* newChild, Node* oldChild);

#endif /* Node_h */
