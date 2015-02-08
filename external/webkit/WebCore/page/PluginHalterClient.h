

#ifndef PluginHalterClient_h
#define PluginHalterClient_h

namespace WebCore {

class Node;
class String;

class PluginHalterClient {
public:
    virtual ~PluginHalterClient() { }

    virtual bool shouldHaltPlugin(Node*, bool isWindowed, const String& pluginName) const = 0;
    virtual bool enabled() const = 0;
};

} // namespace WebCore

#endif // PluginHalterClient_h
