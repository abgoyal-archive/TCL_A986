

#include "NodeList.h"

#include <stdlib.h>

extern NodeList* NodeList_new(Node* parentNode)
{
    Node_ref(parentNode);

    NodeList* nodeList = (NodeList*)malloc(sizeof(NodeList));
    nodeList->parentNode = parentNode;
    nodeList->refCount = 0;
    return nodeList;
}

extern unsigned NodeList_length(NodeList* nodeList)
{
    /* Linear count from tail -- good enough for our purposes here */
    unsigned i = 0;
    NodeLink* n = nodeList->parentNode->childNodesTail;
    while (n) {
        n = n->prev;
        ++i;
    }

    return i;
}

extern Node* NodeList_item(NodeList* nodeList, unsigned index)
{
    unsigned length = NodeList_length(nodeList);
    if (index >= length)
        return NULL;

    /* Linear search from tail -- good enough for our purposes here */
    NodeLink* n = nodeList->parentNode->childNodesTail;
    unsigned i = 0;
    unsigned count = length - 1 - index;
    while (i < count) {
        ++i;
        n = n->prev;
    }
    return n->node;
}

extern void NodeList_ref(NodeList* nodeList)
{
    ++nodeList->refCount;
}

extern void NodeList_deref(NodeList* nodeList)
{
    if (--nodeList->refCount == 0) {
        Node_deref(nodeList->parentNode);
        free(nodeList);
    }
}
