

#ifndef WDS_DEVICE_H
#define WDS_DEVICE_H

#include <stdlib.h>

class AdbConnection;

class Device {
public:
    // Type of device.
    // TODO: Add simulator support
    enum DeviceType {
        NONE = -1,
        EMULATOR,
        DEVICE
    };

    // Takes ownership of name
    Device(char* name, DeviceType type, const AdbConnection* conn)
        : m_connection(conn)
        , m_name(strdup(name))
        , m_type(type) {}
    ~Device() { free(m_name); }

    const char* name() const  { return m_name; }
    DeviceType type() const   { return m_type; }

    // Send a request to this device.
    bool sendRequest(const char* req) const;

private:
    const AdbConnection* m_connection;
    char*      m_name;
    DeviceType m_type;
};

#endif
