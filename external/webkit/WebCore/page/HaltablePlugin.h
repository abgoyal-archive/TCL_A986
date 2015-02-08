

#ifndef HaltablePlugin_h
#define HaltablePlugin_h

namespace WebCore {

class Node;

class HaltablePlugin {
public:
    virtual ~HaltablePlugin() { }

    virtual void halt() = 0;
    virtual void restart() = 0;
    virtual Node* node() const = 0;
    virtual bool isWindowed() const = 0;
    virtual String pluginName() const = 0;
};

} // namespace WebCore

#endif // HaltablePlugin_h
