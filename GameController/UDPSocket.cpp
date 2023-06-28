#include "UDPSocket.hpp"

UDPSocket::UDPSocket(int port, char *address, bool Datagram = true, bool broadcast, bool reuseSock = true, bool isServer = false, int timeout = 0)
{
    connected = false;
    received[0] = '\0';

    sockaddr_in addr;
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    int optVal = 1;

    if (reuseSock)
        retval = setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &optVal, sizeof(optVal));

    if (timeout > 0)
    {
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = timeout * 1000;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    }

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);

    if (isServer)
    {
        client = -1;
        retval = bind(sock, (struct sockaddr *)&addr, sizeof(addr));
        retval = listen(sock, 1);
        connected = false;
    }
    else
    {
        client = -1;
        memset(&outaddr, 0, sizeof(outaddr));
        outaddr.sin_family = AF_INET;
        outaddr.sin_addr.s_addr = inet_addr(address);
        outaddr.sin_port = htons(port);
    }

    if (broadcast)
    {
        retval = setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &optVal, sizeof(optVal));
        retval = bind(sock, (struct sockaddr *)&addr, sizeof(addr));
    }
    else
    {
        retval = connect(sock, (struct sockaddr *)&addr, sizeof(addr));
    }

    connected = retval != 0;

    if (connected)
        std::cout << "Connected" << std::endl;
}

UDPSocket::~UDPSocket() {
    close(sock);
}

int UDPSocket::getRetVal() {
    return retval;
}

bool UDPSocket::isConnected() {
    return connected;
}

bool UDPSocket::wait() {
    if (client = -1) return true;
    unsigned int clientLen = sizeof(outaddr);
    client = accept(sock, (struct sockaddr *)&outaddr, &clientLen);
    return client >= 0;
}

void UDPSocket::disconnect() {
    if (client != -1) close(client);
    client = -1;
}

void UDPSocket::getAddress(const char* name, char* addr) {
    struct hostent *hp;
    // TODO: gethostbyname() depracated
}