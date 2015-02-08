

#include "ClientUtils.h"
#include <arpa/inet.h>
#include <string.h>

void createTcpSocket(sockaddr_in& addr, short port) {
    memset(&addr, 0, sizeof(sockaddr_in));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
}
