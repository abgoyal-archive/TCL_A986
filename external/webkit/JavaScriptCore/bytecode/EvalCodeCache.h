

#ifndef EvalCodeCache_h
#define EvalCodeCache_h

#include "Executable.h"
#include "JSGlobalObject.h"
#include "Nodes.h"
#include "Parser.h"
#include "SourceCode.h"
#include "UString.h"
#include <wtf/HashMap.h>
#include <wtf/RefPtr.h>

namespace JSC {

    class EvalCodeCache {
    public:
        PassRefPtr<EvalExecutable> get(ExecState* exec, const UString& evalSource, ScopeChainNode* scopeChain, JSValue& exceptionValue)
        {
            RefPtr<EvalExecutable> evalExecutable;

            if (evalSource.size() < maxCacheableSourceLength && (*scopeChain->begin())->isVariableObject())
                evalExecutable = m_cacheMap.get(evalSource.rep());

            if (!evalExecutable) {
                evalExecutable = EvalExecutable::create(exec, makeSource(evalSource));
                exceptionValue = evalExecutable->compile(exec, scopeChain);
                if (exceptionValue)
                    return 0;

                if (evalSource.size() < maxCacheableSourceLength && (*scopeChain->begin())->isVariableObject() && m_cacheMap.size() < maxCacheEntries)
                    m_cacheMap.set(evalSource.rep(), evalExecutable);
            }

            return evalExecutable.release();
        }

        bool isEmpty() const { return m_cacheMap.isEmpty(); }

    private:
        static const int maxCacheableSourceLength = 256;
        static const int maxCacheEntries = 64;

        typedef HashMap<RefPtr<UString::Rep>, RefPtr<EvalExecutable> > EvalCacheMap;
        EvalCacheMap m_cacheMap;
    };

} // namespace JSC

#endif // EvalCodeCache_h
