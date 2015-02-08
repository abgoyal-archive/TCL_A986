

#ifndef Connection_h
#define Connection_h

#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {

class Connection : public RefCounted<Connection> {
public:
    enum ConnectionType {
        Unknown = 0,
        Ethernet = 1,
        WiFi = 2,
        Cell_2G = 3,
        Cell_3G = 4,
    };

    static PassRefPtr<Connection> create() { return adoptRef(new Connection()); }

    ConnectionType type() const;

private:
    Connection() { }
};

} // namespace WebCore

#endif // Connection_h
