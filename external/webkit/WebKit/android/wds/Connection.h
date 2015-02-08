

#ifndef WDS_CONNECTION_H
#define WDS_CONNECTION_H

#include <unistd.h>
#include <sys/socket.h>

namespace android {

namespace WDS {

class Socket {
public:
    Socket(): m_fd(-1) {}
    Socket(int fd): m_fd(fd) {}
    ~Socket() {
        if (m_fd != -1) {
            shutdown(m_fd, SHUT_RDWR);
            close(m_fd);
        }
    }
    // Open a new socket using PF_INET and SOCK_STREAM
    bool open();
    int fd() const { return m_fd; }
private:
    int m_fd;
};

class Connection {
public:
    Connection(int conn): m_socket(conn) {}
    int read(char buf[], size_t length) const {
        return recv(m_socket.fd(), buf, length, 0);
    }
    int write(const char buf[], size_t length) const {
        return send(m_socket.fd(), buf, length, 0);
    }
    int write(const char buf[]) const {
        return write(buf, strlen(buf));
    }
private:
    Socket m_socket;
};

class ConnectionServer {
public:
    ConnectionServer() {}

    // Establish a connection to the local host on the given port.
    bool connect(short port);

    // Blocks on the established socket until a new connection arrives.
    Connection* accept() const;
private:
    Socket m_socket;
};

} // end namespace WDS

} // end namespace android

#endif
