

#ifndef JAVA_SHARED_CLIENT_H
#define JAVA_SHARED_CLIENT_H

namespace android {

    class TimerClient;
    class CookieClient;
    class PluginClient;
    class KeyGeneratorClient;

    class JavaSharedClient
    {
    public:
        static TimerClient* GetTimerClient(); 
        static CookieClient* GetCookieClient();
        static PluginClient* GetPluginClient();
        static KeyGeneratorClient* GetKeyGeneratorClient();

        static void SetTimerClient(TimerClient* client);
        static void SetCookieClient(CookieClient* client);
        static void SetPluginClient(PluginClient* client);
        static void SetKeyGeneratorClient(KeyGeneratorClient* client);

        // can be called from any thread, to be executed in webkit thread
        static void EnqueueFunctionPtr(void (*proc)(void*), void* payload);
        // only call this from webkit thread
        static void ServiceFunctionPtrQueue();

    private:
        static TimerClient* gTimerClient;
        static CookieClient* gCookieClient;
        static PluginClient* gPluginClient;
        static KeyGeneratorClient* gKeyGeneratorClient;
    };
}
#endif
