

#ifndef SelectorNodeList_h
#define SelectorNodeList_h

#include "StaticNodeList.h"

namespace WebCore {

    class CSSSelectorList;

    PassRefPtr<StaticNodeList> createSelectorNodeList(Node* rootNode, const CSSSelectorList&);

} // namespace WebCore

#endif // SelectorNodeList_h
