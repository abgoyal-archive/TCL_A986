

#include "config.h"

#include "Connection.h"

#include "NetworkStateNotifier.h"

namespace WebCore {

Connection::ConnectionType Connection::type() const
{
    return networkStateNotifier().type();
}

};