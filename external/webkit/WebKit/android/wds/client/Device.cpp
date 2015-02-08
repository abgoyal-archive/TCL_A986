

#include "AdbConnection.h"
#include "Device.h"

bool Device::sendRequest(const char* req) const {
    return m_connection->sendRequest("host-serial:%s:%s", m_name, req);
}
