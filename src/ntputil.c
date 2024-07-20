#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "ntputil.h"
#include <time.h>
// This is a simple NTP client implimentation for the sBitx that will check the current computer time against the defined ntp server pool and adjust 
// the local time on the machine accordingly if it is +/- 1 second out of sync.
// 6/30/24 W2JON

#define NTP_TIMESTAMP_DELTA (2208988800ull)

#pragma pack(1)
struct ntp_packet {
    uint8_t li_vn_mode;
    uint8_t stratum;
    uint8_t poll;
    uint8_t precision;
    uint32_t rootDelay;
    uint32_t rootDispersion;
    uint32_t refId;
    uint32_t refTm_s;
    uint32_t refTm_f;
    uint32_t origTm_s;
    uint32_t origTm_f;
    uint32_t rxTm_s;
    uint32_t rxTm_f;
    uint32_t txTm_s;
    uint32_t txTm_f;
};
#pragma pack(0)

long getaddress(const char* host) {
    int i, dotcount = 0;
    char* p = (char*)host;
    struct hostent* pent;

    while (*p) {
        for (i = 0; i < 3; i++, p++)
            if (!isdigit(*p))
                break;
        if (*p != '.')
            break;
        p++;
        dotcount++;
    }

    if (dotcount == 3 && i > 0 && i <= 3)
        return inet_addr(host);

    pent = gethostbyname(host);
    if (!pent)
        return 0;

    return *((long*)(pent->h_addr));
}

int ntp_request(const char* ntp_server) {
    struct sockaddr_in addr;
    int retryAfter = 500, i, len, ret;
    int nretries = 10;
    int sock;
    struct timeval tv;
    fd_set fd;
    struct ntp_packet reply;

    printf("Resolving NTP server at %s\n", ntp_server);
    uint32_t address = getaddress(ntp_server);
    if (!address) {
        printf("NTP server is not reachable right now\n");
        return -1;
    }

    struct ntp_packet request;
    memset(&request, 0, sizeof(struct ntp_packet));
    request.li_vn_mode = 0x1b;

    sock = (int)socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    addr.sin_addr.s_addr = address;
    addr.sin_port = htons(123);
    addr.sin_family = AF_INET;

    ret = connect(sock, (struct sockaddr*)&addr, sizeof(addr));
    if (ret < 0) {
        perror("connect");
        return -1;
    }

    ret = send(sock, &request, sizeof(struct ntp_packet), 0);
    if (ret < 0) {
        perror("send");
        close(sock);
        return -1;
    }

    tv.tv_sec = retryAfter / 1000;
    tv.tv_usec = (retryAfter % 1000) * 1000;
    FD_ZERO(&fd);
    FD_SET(sock, &fd);

    ret = select(sock + 1, &fd, NULL, NULL, &tv);
    if (ret <= 0) {
        puts("Timeout or select error\n");
        close(sock);
        return -1;
    }

    len = sizeof(addr);
    ret = recv(sock, &reply, sizeof(struct ntp_packet), 0);
    if (ret <= 0) {
        puts("recvfrom error\n");
        close(sock);
        return -1;
    }

    close(sock);

    reply.txTm_s = ntohl(reply.txTm_s);
    reply.txTm_f = ntohl(reply.txTm_f);
    time_t txTm = (time_t)(reply.txTm_s - NTP_TIMESTAMP_DELTA);
    struct tm* utc = gmtime(&txTm);

    printf("Time: %d-%d-%d %02d:%02d:%02d\n", utc->tm_year + 1900, utc->tm_mon + 1, utc->tm_mday,
        utc->tm_hour, utc->tm_min, utc->tm_sec);

    return txTm;
}

void sync_system_time(const char* ntp_server) {
    time_t current_time;
    time(&current_time);  // Get current system time

    time_t ntp_time = ntp_request(ntp_server);
    if (ntp_time == -1) {
        printf("Failed to synchronize system time with NTP server.\n");
        return;
    }

    // Check if adjustment is needed (within 1 second difference)
    if (labs(current_time - ntp_time) > 1) {
        // Adjust system time
        struct timeval tv;
        tv.tv_sec = ntp_time;
        tv.tv_usec = 0;

        if (settimeofday(&tv, NULL) < 0) {
            perror("settimeofday");
            printf("Failed to adjust system time.\n");
        }
        else {
            printf("System time adjusted to match NTP time.\n");
        }
    }
    else {
        printf("System time is already within 1 second of NTP time.\n");
    }
}
