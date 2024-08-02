//gcc -o sbitx sbitx.c sdr_server.c -lfftw3 -lm -lpthread
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <fftw3.h>  // Make sure to include the FFTW library

#define FFT_PORT 12345  // Port to listen for SDRConsole connection
#define MAX_BINS 2048  // Adjust according to your FFT size

extern fftw_complex *fft_in, *fft_out, *fft_spectrum;
extern fftw_plan plan_fwd;

int server_sockfd, client_sockfd;  // Server and client socket file descriptors
struct sockaddr_in server_addr, client_addr;

// Server initialization function
void initialize_server() {
    server_sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_sockfd < 0) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(FFT_PORT);
    server_addr.sin_addr.s_addr = inet_addr("192.168.4.187");  // Bind to radio's IP

    if (bind(server_sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        perror("bind failed");
        close(server_sockfd);
        exit(EXIT_FAILURE);
    }

    if (listen(server_sockfd, 5) < 0) {
        perror("listen failed");
        close(server_sockfd);
        exit(EXIT_FAILURE);
    }

    printf("Server initialized. Waiting for SDRConsole to connect...\n");
}

// Function to stream FFT data
void stream_fft_data() {
    socklen_t client_len = sizeof(client_addr);
    client_sockfd = accept(server_sockfd, (struct sockaddr *)&client_addr, &client_len);
    if (client_sockfd < 0) {
        perror("accept failed");
        close(server_sockfd);
        exit(EXIT_FAILURE);
    }

    printf("SDRConsole connected.\n");

    while (1) {
        fftw_execute(plan_fwd);  // Execute the FFT

        ssize_t sent = send(client_sockfd, fft_out, sizeof(fftw_complex) * MAX_BINS, 0);
        if (sent < 0) {
            perror("send failed");
            break;
        }

        usleep(1000);  // Adjust delay to control bandwidth
    }

    close(client_sockfd);  // Close the client socket when done
}

void *start_streaming(void *arg) {
    stream_fft_data();
    return NULL;
}
