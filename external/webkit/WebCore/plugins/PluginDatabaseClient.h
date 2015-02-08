
#ifndef PluginDatabaseClient_h
#define PluginDatabaseClient_h

namespace WebCore {

    class PluginPackage;
    class String;

    class PluginDatabaseClient {
    public:
        virtual ~PluginDatabaseClient() { }
        virtual bool shouldLoadPluginAtPath(const String&) = 0;
        virtual bool shouldLoadPluginPackage(const PluginPackage*) = 0;
    };

} // namespace WebCore

#endif // PluginDatabaseClient_h
