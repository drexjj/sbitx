#include <stdint.h>

// Protocol constants
#define HPSDR_MCAST_ADDR        "255.255.255.255"
#define HPSDR_PORT              1024
#define HPSDR_PKT_SIZE          1032
#define HPSDR_DISCOVERY_REPLY   60
#define SAMPLES_PER_PKT         126

// Packet types returned by hpsdr_classify()
#define PKT_UNKNOWN   0
#define PKT_DISCOVERY 1
#define PKT_START     2
#define PKT_STOP      3
#define PKT_EP2       4

typedef struct {
    int      mox;
    uint32_t freq;                      // RX NCO freq (addr 0x02)
    uint32_t tx_freq;                   // TX NCO freq (addr 0x01), 0 if not present
    int      n_samples;
    float    iq[SAMPLES_PER_PKT * 2];
} hpsdr_ep2_result_t;

// Lifecycle
int  hpsdr_init(void);
void hpsdr_stop(void);
void hpsdr_poll(void);
int  hpsdr_is_connected(void);

// RX: push 96 kHz IQ from the sBitx audio thread toward the SDR app
void hpsdr_send_iq(double *i_samples, double *q_samples, int n);

// TX: pull 96 kHz upsampled IQ from the SDR app into the sBitx audio thread
// Returns 1 if a remote app is actively sending TX IQ data
int  hpsdr_tx_iq_active(void);
// Retrieve up to max_samples 96 kHz TX IQ pairs; returns count actually written
int  hpsdr_get_tx_iq(double *out_i, double *out_q, int max_samples);
