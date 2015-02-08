

#include "Node.h"
#include <stddef.h>
#include <stdlib.h>

Node* Node_new(void)
{
    Node* node = (Node*)malloc(sizeof(Node));
    node->refCount = 0;
    node->nodeType = "Node";
    node->childNodesTail = NULL;
    
    return node;
}

void Node_appendChild(Node* node, Node* child)
{
    Node_ref(child);
    NodeLink* nodeLink = (NodeLink*)malloc(sizeof(NodeLink));
    nodeLink->node = child;
    nodeLink->prev = node->childNodesTail;
    node->childNodesTail = nodeLink;
}

void Node_removeChild(Node* node, Node* child)
{
    /* Linear search from tail -- good enough for our purposes here */
    NodeLink* current;
    NodeLink** currentHandle;
    for (currentHandle = &node->childNodesTail, current = *currentHandle; current; currentHandle = &current->prev, current = *currentHandle) {
        if (current->node == child) {
            Node_deref(current->node);
            *currentHandle = current->prev;
            free(current);
            break;
        }
    }
}

void Node_replaceChild(Node* node, Node* newChild, Node* oldChild)
{
    /* Linear search from tail -- good enough for our purposes here */
    NodeLink* current;
    for (current = node->childNodesTail; current; current = current->prev) {
        if (current->node == oldChild) {
            Node_deref(current->node);
            current->node = newChild;
        }
    }
}

void Node_ref(Node* node)
{
    ++node->refCount;
}

void Node_deref(Node* node)
{
    if (--node->refCount == 0)
        free(node);
}
