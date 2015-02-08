

#ifndef WDS_CLIENT_UTILS_H
#define WDS_CLIENT_UTILS_H

#include <arpa/inet.h>

#ifdef __FreeBSD__
#include <netinet/in.h>
#include <sys/socket.h>
#endif

// Callers need to include Log.h and errno.h to use this macro
#define log_errno(str) LOGE("%s: %s", str, strerror(errno))

// Fill in the sockaddr_in structure for binding to the localhost on the given
// port
void createTcpSocket(sockaddr_in& addr, short port);

#endif
