#include "qemu/osdep.h"
#include "qemu-common.h"
#include <stdio.h>
#include <stdlib.h>
#include <netdb.h>
#include <netinet/in.h>
#include <string.h>
#include <poll.h>
#include "sys/un.h"
#include "nrf51.h"

static int sockfd;
static int clientfd;

int nrf51_uart_comm_write(char *pchBuf, int nLen)
{
    struct pollfd pfd;
    if (!clientfd)
        return -1;

    pfd.fd = clientfd;
    pfd.events = POLLOUT;

    if (poll(&pfd, 1, 0) < 1)
    {
        //No fds are ready.
        return -1;
    }

    if (pfd.events & POLLOUT)
    {
        return write(clientfd, pchBuf, nLen);
    }

    return 0;
}

int nrf51_uart_comm_read(char *pchBuf, int nAvailable)
{
    int n;
    struct pollfd pfd;

    //TODO: move accept code into separate function?
    //This will require comm_read to be called even if we don't need to 'read'.
    if (!clientfd)
    {
        pfd.fd = sockfd;
        pfd.events = POLLIN;
        if (poll(&pfd, 1, 0) < 1)
        {
            //no clients waiting.
            return -1;
        }

        if (pfd.events & POLLIN)
        {
            printf("new client\n");
            clientfd = accept(sockfd, 0, 0);
        }
        else
        {
            return -1;
        }
    }

    pfd.fd = clientfd;
    pfd.events = POLLIN;

    if (poll(&pfd, 1, 0) < 1)
    {
        //No fd is ready.
        return 0;
    }

    if (pfd.events & POLLIN)
    {
        n = read(clientfd, pchBuf, nAvailable);
    }
    else
    {
        printf("Unexpected event: %d", (int)pfd.events);
        goto err_sock;
        //return 0;
    }

    if (n < 1)
    {
err_sock:
        perror("ERROR reading from socket");
        close(clientfd);
        clientfd = 0;
        return -1;
    }

    return n;
}

int nrf51_uart_comm_init(void)
{
    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    char path[64];

    snprintf(path, sizeof(path), "/tmp/nrf51_%04x.sock", nrf_id);

    if(!unlink(path) && errno != ENOENT)
    {
        perror("Cannot remove UNIX socket file");
    }

    strncpy(addr.sun_path, path, sizeof(addr.sun_path) - 1);

    sockfd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (sockfd < 0)
    {
        perror("opening stream socket");
        return -1;
    }

    if (bind(sockfd, (struct sockaddr *)&addr, sizeof(addr)))
    {
        perror("error bind socket");
        return -1;
    }

    return listen(sockfd, 3);
}
