

#ifndef APIShims_h
#define APIShims_h

#include "CallFrame.h"
#include "JSLock.h"

namespace JSC {

class APIEntryShimWithoutLock {
protected:
    APIEntryShimWithoutLock(JSGlobalData* globalData, bool registerThread)
        : m_globalData(globalData)
        , m_entryIdentifierTable(setCurrentIdentifierTable(globalData->identifierTable))
    {
        if (registerThread)
            globalData->heap.registerThread();
        m_globalData->timeoutChecker.start();
    }

    ~APIEntryShimWithoutLock()
    {
        m_globalData->timeoutChecker.stop();
        setCurrentIdentifierTable(m_entryIdentifierTable);
    }

private:
    JSGlobalData* m_globalData;
    IdentifierTable* m_entryIdentifierTable;
};

class APIEntryShim : public APIEntryShimWithoutLock {
public:
    // Normal API entry
    APIEntryShim(ExecState* exec, bool registerThread = true)
        : APIEntryShimWithoutLock(&exec->globalData(), registerThread)
        , m_lock(exec)
    {
    }

    // JSPropertyNameAccumulator only has a globalData.
    APIEntryShim(JSGlobalData* globalData, bool registerThread = true)
        : APIEntryShimWithoutLock(globalData, registerThread)
        , m_lock(globalData->isSharedInstance ? LockForReal : SilenceAssertionsOnly)
    {
    }

private:
    JSLock m_lock;
};

class APICallbackShim {
public:
    APICallbackShim(ExecState* exec)
        : m_dropAllLocks(exec)
        , m_globalData(&exec->globalData())
    {
        resetCurrentIdentifierTable();
        m_globalData->timeoutChecker.start();
    }

    ~APICallbackShim()
    {
        m_globalData->timeoutChecker.stop();
        setCurrentIdentifierTable(m_globalData->identifierTable);
    }

private:
    JSLock::DropAllLocks m_dropAllLocks;
    JSGlobalData* m_globalData;
};

}

#endif
