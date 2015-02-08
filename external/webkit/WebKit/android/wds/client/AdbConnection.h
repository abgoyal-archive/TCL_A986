

#ifndef WDS_ADB_CONNECTION_H
#define WDS_ADB_CONNECTION_H

#include "DeviceList.h"

class AdbConnection {
public:
    AdbConnection() : m_fd(-1) {}
    ~AdbConnection() { clearDevices(); }
    void close();
    bool connect();
    bool sendRequest(const char* fmt, ...) const;
    const DeviceList& getDeviceList();

private:
    bool checkOkayResponse() const;
    void clearDevices();
    DeviceList m_devices;
    int m_fd;
};

#endif
