#ifndef UDPSOCKET_H
#define UDPSOCKET_H

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netdb.h>
#include <string.h>
#include <iostream>

#define TRUE 1
#define FALSE 0

class UDPSocket {
private: 
    int sock;
    int client;

    bool connected;
    long retval;
    sockaddr_in outaddr;
    char ip[30];
    char received[30];

public:
    UDPSocket(int port, char* address, bool Datagram = true, bool broadcast, bool reuseSock = true, bool isServer = false, int timeout = 0);
    ~UDPSocket();

    int getRetVal();
    bool isConnected();

    bool wait();
    void disconnect();

    long receive(char* msg, int msgSize);
    char* received_from();
    long send(const char* msg, int msgSize);
    long sentTo(const char* msg, int msgSize, const char* name);
    int getAddress(const char* name, char* addr);
    const char* getAddress(const char* name);
};

#endif