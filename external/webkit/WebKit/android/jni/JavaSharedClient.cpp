

#include "config.h"
#include "JavaSharedClient.h"
#include "TimerClient.h"
#include "SkDeque.h"
#include "SkThread.h"

namespace android {
    TimerClient* JavaSharedClient::GetTimerClient()
    {
        return gTimerClient;
    }

    CookieClient* JavaSharedClient::GetCookieClient()
    {
        return gCookieClient;
    }

    PluginClient* JavaSharedClient::GetPluginClient()
    {
        return gPluginClient;
    }

    KeyGeneratorClient* JavaSharedClient::GetKeyGeneratorClient()
    {
        return gKeyGeneratorClient;
    }

    void JavaSharedClient::SetTimerClient(TimerClient* client)
    {
        gTimerClient = client;
    }

    void JavaSharedClient::SetCookieClient(CookieClient* client)
    {
        gCookieClient = client;
    }

    void JavaSharedClient::SetPluginClient(PluginClient* client)
    {
        gPluginClient = client;
    }

    void JavaSharedClient::SetKeyGeneratorClient(KeyGeneratorClient* client)
    {
        gKeyGeneratorClient = client;
    }

    TimerClient*    JavaSharedClient::gTimerClient = NULL;
    CookieClient*   JavaSharedClient::gCookieClient = NULL;
    PluginClient*   JavaSharedClient::gPluginClient = NULL;
    KeyGeneratorClient* JavaSharedClient::gKeyGeneratorClient = NULL;

    ///////////////////////////////////////////////////////////////////////////
    
    struct FuncPtrRec {
        void (*fProc)(void* payload);
        void* fPayload;
    };
    
    static SkMutex gFuncPtrQMutex;
    static SkDeque gFuncPtrQ(sizeof(FuncPtrRec));

    void JavaSharedClient::EnqueueFunctionPtr(void (*proc)(void* payload),
                                              void* payload)
    {
        gFuncPtrQMutex.acquire();

        FuncPtrRec* rec = (FuncPtrRec*)gFuncPtrQ.push_back();
        rec->fProc = proc;
        rec->fPayload = payload;
        
        gFuncPtrQMutex.release();
        
        gTimerClient->signalServiceFuncPtrQueue();
    }

    void JavaSharedClient::ServiceFunctionPtrQueue()
    {
        for (;;) {
            void (*proc)(void*) = 0;
            void* payload = 0;
            const FuncPtrRec* rec;
            
            // we have to copy the proc/payload (if present). we do this so we
            // don't call the proc inside the mutex (possible deadlock!)
            gFuncPtrQMutex.acquire();
            rec = (const FuncPtrRec*)gFuncPtrQ.front();
            if (rec) {
                proc = rec->fProc;
                payload = rec->fPayload;
                gFuncPtrQ.pop_front();
            }
            gFuncPtrQMutex.release();
            
            if (!rec)
                break;
            proc(payload);
        }
    }
}
