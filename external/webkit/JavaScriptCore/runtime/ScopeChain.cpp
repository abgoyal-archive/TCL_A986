

#include "config.h"
#include "ScopeChain.h"

#include "JSActivation.h"
#include "JSGlobalObject.h"
#include "JSObject.h"
#include "PropertyNameArray.h"
#include <stdio.h>

namespace JSC {

#ifndef NDEBUG

void ScopeChainNode::print() const
{
    ScopeChainIterator scopeEnd = end();
    for (ScopeChainIterator scopeIter = begin(); scopeIter != scopeEnd; ++scopeIter) {
        JSObject* o = *scopeIter;
        PropertyNameArray propertyNames(globalObject->globalExec());
        o->getPropertyNames(globalObject->globalExec(), propertyNames);
        PropertyNameArray::const_iterator propEnd = propertyNames.end();

        fprintf(stderr, "----- [scope %p] -----\n", o);
        for (PropertyNameArray::const_iterator propIter = propertyNames.begin(); propIter != propEnd; propIter++) {
            Identifier name = *propIter;
            fprintf(stderr, "%s, ", name.ascii());
        }
        fprintf(stderr, "\n");
    }
}

#endif

int ScopeChain::localDepth() const
{
    int scopeDepth = 0;
    ScopeChainIterator iter = this->begin();
    ScopeChainIterator end = this->end();
    while (!(*iter)->inherits(&JSActivation::info)) {
        ++iter;
        if (iter == end)
            break;
        ++scopeDepth;
    }
    return scopeDepth;
}

} // namespace JSC
