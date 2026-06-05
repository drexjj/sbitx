// hpsdr_p1.c — HPSDR Protocol 1 interface for sBitx
//
// Provides the interface between the sBitx and an external SDR application
// using HPSDR Protocol 1, emulating a HermesLite radio.  With no SDR app present, 
// the module does nothing but listen on the UDP socket — completely invisible to
// normal sBitx operation.
//
// Data flow:
//   RX (sBitx → SDR app):  audio thread → hpsdr_send_iq() → [96k→48k decimation]
//                           → iq_buf → build_and_send_packet() → UDP/EP6 → SDR app
//   TX (SDR app → sBitx):  UDP/EP2 → hpsdr_unpack_ep2() → handle_command() → tx_upsample_and_push()
//                           → [48k→96k upsampling] → tx_iq_ring → hpsdr_get_tx_iq() → audio thread
//
// Major sections:
//  1. RX Signal Processing:  96k→48k half-band decimation, EP6 frame staging
//  2. TX Signal Processing;  48k→96k polyphase upsampling, ring buffer, public API
//  3. HPSDR Protocol 1:      packet I/O: classify, pack/unpack EP2/EP6, session control
//  4. sBitx State Integration:  translate HPSDR state into sBitx core calls (freq, T/R)
//  5. Initialization, Control & Shutdown:  socket setup, poll thread, watchdog
//
// There is support for data moving in both directions but the focus has been on receive functions.
//
// Inspired by Dave N1AI and Juan WP3DN
// Mike KB2ML

// System
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <stdatomic.h>

// Networking
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

// UI/Framework
#include <gtk/gtk.h>

// Local
#include "hpsdr_p1.h"

// -----------------------------------------------------------------------------
// Forward Declarations
// -----------------------------------------------------------------------------

// Shared utility
static unsigned long millis_now(void);

// Section 1: RX Signal Processing
static void rx_filter_and_decimate(double i0, double i1, double q0, double q1);
// Public: hpsdr_send_iq (defined in .h)

// Section 2: TX Signal Processing
static void flush_tx_ring(void);
static void tx_upsample_and_push(double i_val, double q_val);
// Public: hpsdr_tx_iq_active, hpsdr_get_tx_iq (defined in .h)

// Section 3: HPSDR Protocol 1
static int  hpsdr_classify(const uint8_t *buf, int len);
static void hpsdr_build_discovery_reply(uint8_t *reply, int in_use);
static int  hpsdr_unpack_ep2(const uint8_t *buf, int len, hpsdr_ep2_result_t *result);
static void build_and_send_packet(void);
static void reset_all_tx_state(void);
static void handle_command(uint8_t *buf, int len, struct sockaddr_in *sender);

// Section 4: sBitx State Integration
static void     apply_freq_from_ep2(uint32_t freq);
static void     apply_mox_from_ep2(int mox);
static gboolean hpsdr_tr_idle(gpointer data);

// Section 5: Initialization, Control & Shutdown
static gboolean hpsdr_watchdog(gpointer data);
static void    *hpsdr_poll_thread(void *arg);
// Public: hpsdr_init, hpsdr_stop, hpsdr_is_connected, hpsdr_poll (defined in .h)

// -----------------------------------------------------------------------------
// Compile-time constants (protocol-independent)
// -----------------------------------------------------------------------------
#define TX_SOFT 2   // trigger code for software-initiated TX (passed to tx_on())

// -----------------------------------------------------------------------------
// Externs — sBitx core symbols this module drives
// -----------------------------------------------------------------------------
extern void remote_execute(char *command);
extern int  freq_hdr;
extern int  in_tx;
extern void tx_on(int trigger);
extern void tx_off(void);
extern void cmd_exec(char *cmd);

// -----------------------------------------------------------------------------
// Shared statics — variables accessed by two or more sections
//
//   Networking / session:
//     hpsdr_sock         — UDP socket fd; -1 when not open
//     stream_dest        — address/port of the currently connected SDR client
//     client_active      — 1 while a client session (START…STOP) is open
//     running            — 1 while the poll thread should keep looping
//
//   Protocol / DSP shared state:
//     hpsdr_sample_rate  — rate negotiated in EP2 addr 0x00 (48k or 96k);
//                          read by Section 1 (RX path) and written by Section 3 (EP2 decode)
//     remote_mox         — last committed (debounced) MOX state from SDR app;
//                          written by Section 4, read/reset by Section 3
//     tr_pending         — pending T/R action for GTK idle: 0=none 1=TX 2=RX;
//                          written by Sections 3 and 4, consumed by Section 4 idle callback
//     ep2_last_time_ms   — timestamp of last EP2 packet;
//                          written by Section 3, read by Section 5 watchdog
//     hpsdr_tx_data_active — 1 while remote TX IQ is actively flowing;
//                          written by Sections 3 and 4, read by Sections 2 and 3
// -----------------------------------------------------------------------------
static int                hpsdr_sock            = -1;
static struct sockaddr_in stream_dest;
static volatile int       client_active         = 0;
static volatile int       running               = 0;
static int                hpsdr_sample_rate     = 48000;

static volatile int           remote_mox           = 0;
static volatile int           tr_pending           = 0;
static volatile unsigned long ep2_last_time_ms     = 0;
static volatile int           hpsdr_tx_data_active = 0;

// Last non-zero RX1 (DDC0) frequency seen in EP2 addr 0x02
// persisted because the C&C round-robin only delivers this slot once every
// 19 frames — it is zero in the other 18.  This is the operator's selected
// signal frequency in both SDR Console (shown as "RX 1") and SPARK SDR
// (always at spectrum center).
static uint32_t last_rx_freq = 0;
// Last non-zero TX VFO frequency seen in EP2 addr 0x01.
// In SPARK SDR this is the selected signal frequency (spectrum center).
static uint32_t last_tx_freq = 0;

// -----------------------------------------------------------------------------
// Shared utility
// -----------------------------------------------------------------------------

// Return a monotonic millisecond timestamp. Used for timeouts and watchdogs
// across all sections.
static unsigned long millis_now(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (unsigned long)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

// =============================================================================
// SECTION 1 — RX SIGNAL PROCESSING
// =============================================================================
// accept 96 kHz dual-channel IQ from the sBitx audio thread,
// optionally decimate 2:1 to 48 kHz using a half-band FIR, stage the result
// into iq_buf, and trigger an EP6 packet to the SDR app when the buffer is full.
//
// Data flow:
//   sBitx audio thread
//     → hpsdr_send_iq()              [entry point; selects 48k or 96k path]
//       → rx_filter_and_decimate()   [96k→48k half-band LPF + 2:1 decimation]
//         → iq_buf_i / iq_buf_q      [126-sample staging buffer]
//           → build_and_send_packet()  [triggered when buffer is full → EP6 UDP]
//
// Thread: called exclusively from the sBitx audio thread.

// 126-sample staging buffer — filled by the RX path, drained by build_and_send_packet()
static double iq_buf_i[SAMPLES_PER_PKT];
static double iq_buf_q[SAMPLES_PER_PKT];
static int    iq_buf_count = 0;

// Per-sample gain applied to outbound RX IQ before packing into EP6
static double hpsdr_iq_gain = 1.0;

// 31-tap half-band FIR coefficients (Fs=96k, cutoff=24k).
// Every other tap is 0 except the center tap (index 15 = 0.5).
// Declared static const so the compiler can fold zeros and exploit symmetry.
static const double hb_coeffs[31] = {
    -0.00055,  0.0,  0.00165,  0.0, -0.00411,  0.0,  0.00877,  0.0,
    -0.01736,  0.0,  0.03433,  0.0, -0.07612,  0.0,  0.30338,  0.5,
     0.30338,  0.0, -0.07612,  0.0,  0.03433,  0.0, -0.01736,  0.0,
     0.00877,  0.0, -0.00411,  0.0,  0.00165,  0.0, -0.00055
};

// Circular history buffer for the RX FIR (sized to power-of-two for masking)
static double rx_hist_i[32];
static double rx_hist_q[32];
static int    rx_hist_ptr = 0;

// Apply the 24 kHz half-band LPF to one input sample pair then decimate 2:1.
// i0/q0 is the older sample, i1/q1 is the newer; one output sample is produced.
// Uses FIR symmetry to reduce the 31-tap convolution to 9 multiply-adds.
static void rx_filter_and_decimate(double i0, double i1, double q0, double q1) {
  // Push newest sample pair into the circular history (newer sample first)
  rx_hist_ptr = (rx_hist_ptr - 1) & 31;
  rx_hist_i[rx_hist_ptr] = i1;
  rx_hist_q[rx_hist_ptr] = q1;

  rx_hist_ptr = (rx_hist_ptr - 1) & 31;
  rx_hist_i[rx_hist_ptr] = i0;
  rx_hist_q[rx_hist_ptr] = q0;

  // Center tap (index 15) is 0.5 — apply directly
  int    p      = rx_hist_ptr;
  double filt_i = rx_hist_i[(p + 15) & 31] * 0.5;
  double filt_q = rx_hist_q[(p + 15) & 31] * 0.5;

  // Exploit half-band symmetry: sum symmetric non-zero tap pairs (indices 0,2,...,14)
  // This reduces multiplications from 31 to 9.
  for (int j = 0; j < 15; j += 2) {
    double c       = hb_coeffs[j];
    int    idx_low = (p + j)      & 31;
    int    idx_hi  = (p + 30 - j) & 31;
    filt_i += (rx_hist_i[idx_low] + rx_hist_i[idx_hi]) * c;
    filt_q += (rx_hist_q[idx_low] + rx_hist_q[idx_hi]) * c;
  }

  // Stage the decimated sample; flush to EP6 when the buffer is full
  iq_buf_i[iq_buf_count] = filt_i * hpsdr_iq_gain;
  iq_buf_q[iq_buf_count] = filt_q * hpsdr_iq_gain;
  iq_buf_count++;

  if (iq_buf_count >= SAMPLES_PER_PKT) {
    build_and_send_packet();
    iq_buf_count = 0;
  }
}

// Entry point called by the sBitx audio thread with 96 kHz IQ samples.
// Selects the 48 kHz decimation path or the 96 kHz pass-through based on
// the sample rate negotiated with the SDR app.
void hpsdr_send_iq(double *i_samples, double *q_samples, int n) {
  if (!client_active || hpsdr_sock < 0) return;

  if (hpsdr_sample_rate == 48000) {
    // 2:1 decimation path: feed sample pairs into the half-band filter
    for (int k = 0; k < n - 1; k += 2) {
      rx_filter_and_decimate(i_samples[k], i_samples[k + 1],
                             q_samples[k], q_samples[k + 1]);
    }
  } else {
    // 96k, 192k, or 384k: pass through at native sBitx rate
    for (int k = 0; k < n; k++) {
        iq_buf_i[iq_buf_count] = i_samples[k] * hpsdr_iq_gain;
        iq_buf_q[iq_buf_count] = q_samples[k] * hpsdr_iq_gain;
        iq_buf_count++;
        if (iq_buf_count >= SAMPLES_PER_PKT) {
            build_and_send_packet();
            iq_buf_count = 0;
        }
    }
  }
}

// =============================================================================
// SECTION 2 — TX SIGNAL PROCESSING
// =============================================================================
// accept 48 kHz TX IQ from the SDR app (via hpsdr_unpack_ep2),
// upsample 2:1 to 96 kHz using a 6-tap polyphase FIR, and store the result in
// a lock-free ring buffer for consumption by the sBitx audio thread.
//
// Data flow:
//   SDR app (UDP/EP2)
//     → hpsdr_unpack_ep2()       [Section 3 — extracts raw 48k IQ samples]
//       → tx_upsample_and_push() [48k→96k polyphase upsampling]
//         → tx_iq_ring_i/q       [lock-free ring buffer, ~85 ms at 96k]
//           → hpsdr_get_tx_iq()  [consumed by sBitx audio thread]
//
// Thread safety: tx_iq_wr is written only by the poll thread; tx_iq_rd is
// written only by the audio thread. Both are _Atomic uint32_t — sufficient for a
// single-producer/single-consumer ring on a cache-coherent architecture.

// Lock-free ring buffer (size must be a power of two)
#define TX_IQ_RING_SIZE 8192
#define TX_IQ_RING_MASK (TX_IQ_RING_SIZE - 1)
static double       tx_iq_ring_i[TX_IQ_RING_SIZE];
static double       tx_iq_ring_q[TX_IQ_RING_SIZE];
static _Atomic uint32_t tx_iq_wr = 0;  // written by poll thread
static _Atomic uint32_t tx_iq_rd = 0;  // written by audio thread

// Declare the TX IQ stream stale if no new data arrives within this window
#define TX_IQ_TIMEOUT_MS 500
static volatile unsigned long tx_iq_last_time_ms = 0;

// Per-sample gain applied to inbound TX IQ during EP2 unpacking (Section 3)
static double hpsdr_tx_gain = 1.0;

// Delay lines for the 6-tap polyphase upsampling FIR
static double tx_hist_i[6] = {0};
static double tx_hist_q[6] = {0};

// Reset the TX ring buffer and clear FIR history.
// Called on session start, MOX-off, and watchdog timeout to prevent stale
// TX samples from leaking into a new transmission.
static void flush_tx_ring(void) {
  atomic_store_explicit(&tx_iq_wr, 0, memory_order_seq_cst);
  atomic_store_explicit(&tx_iq_rd, 0, memory_order_seq_cst);
  memset(tx_hist_i, 0, sizeof(tx_hist_i));
  memset(tx_hist_q, 0, sizeof(tx_hist_q));
}

// Upsample one 48 kHz IQ sample pair to two 96 kHz samples and push both
// into the TX ring buffer.
//
// A 6-tap polyphase half-band FIR generates the interpolated midpoint (Phase 1).
// The aligned original sample (Phase 0) is taken from the center of the delay
// line to maintain phase coherence with the midpoint.
static void tx_upsample_and_push(double i_val, double q_val) {
  // Shift the delay line and insert the new sample
  for (int i = 5; i > 0; i--) {
    tx_hist_i[i] = tx_hist_i[i - 1];
    tx_hist_q[i] = tx_hist_q[i - 1];
  }
  tx_hist_i[0] = i_val;
  tx_hist_q[0] = q_val;

  // 6-tap polyphase coefficients for the interpolated midpoint (Phase 1).
  // (replaced simple linear interpolation)
  static const double taps[6] = {0.0121, -0.0551, 0.2930,
                                  0.2930, -0.0551, 0.0121};

  // Phase 1: FIR-filtered interpolated midpoint
  double mid_i = 0, mid_q = 0;
  for (int i = 0; i < 6; i++) {
    mid_i += tx_hist_i[i] * taps[i];
    mid_q += tx_hist_q[i] * taps[i];
  }

  // Phase 0: aligned original sample (center of delay line)
  double out_i = tx_hist_i[2];
  double out_q = tx_hist_q[2];

   // Drop both samples if the ring is nearly full (overflow guard)
  uint32_t wr = atomic_load_explicit(&tx_iq_wr, memory_order_relaxed);
  uint32_t rd = atomic_load_explicit(&tx_iq_rd, memory_order_acquire);
  if ((uint32_t)(wr - rd) >= (TX_IQ_RING_SIZE - 4))
    return;

  // Write midpoint first, then aligned original
  tx_iq_ring_i[wr & TX_IQ_RING_MASK] = mid_i;
  tx_iq_ring_q[wr & TX_IQ_RING_MASK] = mid_q;
  wr++;

  tx_iq_ring_i[wr & TX_IQ_RING_MASK] = out_i;
  tx_iq_ring_q[wr & TX_IQ_RING_MASK] = out_q;
  wr++;

  // release: guarantees ring data is visible before the new index is
  atomic_store_explicit(&tx_iq_wr, wr, memory_order_release);
  tx_iq_last_time_ms = millis_now();
}

// Returns 1 if the SDR app is actively supplying TX IQ data:
//   - a client session must be open
//   - a TX IQ packet must have arrived within the last TX_IQ_TIMEOUT_MS
//   - the ring buffer must contain at least one sample pair
int hpsdr_tx_iq_active(void) {
  if (!client_active)
    return 0;
  if (millis_now() - tx_iq_last_time_ms > TX_IQ_TIMEOUT_MS)
    return 0;
  uint32_t wr = atomic_load_explicit(&tx_iq_wr, memory_order_acquire);
  uint32_t rd = atomic_load_explicit(&tx_iq_rd, memory_order_relaxed);
  return ((uint32_t)(wr - rd) > 0);
}

int hpsdr_get_tx_iq(double *out_i, double *out_q, int max_samples) {
  uint32_t rd    = atomic_load_explicit(&tx_iq_rd, memory_order_relaxed);
  uint32_t wr    = atomic_load_explicit(&tx_iq_wr, memory_order_acquire);
  uint32_t avail = (uint32_t)(wr - rd);
  int n = ((int)avail < max_samples) ? (int)avail : max_samples;

  for (int k = 0; k < n; k++) {
    out_i[k] = tx_iq_ring_i[(rd + k) & TX_IQ_RING_MASK];
    out_q[k] = tx_iq_ring_q[(rd + k) & TX_IQ_RING_MASK];
  }
  atomic_store_explicit(&tx_iq_rd, rd + (uint32_t)n, memory_order_release);
  return n;
}

// =============================================================================
// SECTION 3 — HPSDR PROTOCOL 1
// =============================================================================
//  all packet-level I/O between this module and the SDR app.
// No sBitx hardware state is changed here; that is delegated to Section 4.
//
// Inbound  (SDR app → sBitx): PKT_DISCOVERY, PKT_START, PKT_STOP, PKT_EP2
// Outbound (sBitx → SDR app): EP6 RX IQ stream
//
// Function order within this section:
//   classify → inbound handlers (discovery, EP2 unpack) →
//   outbound builder (EP6) → session reset → top-level dispatcher

// EP6 sequence counter — incremented with every outbound packet
static uint32_t tx_seq = 0;

// Classify an inbound UDP packet by inspecting its header bytes.
// Returns one of the PKT_* constants defined in hpsdr_p1.h.
static int hpsdr_classify(const uint8_t *buf, int len) {
  if (len < 4 || buf[0] != 0xEF || buf[1] != 0xFE)
    return PKT_UNKNOWN;
  switch (buf[2]) {
    case 0x02: return PKT_DISCOVERY;
    case 0x04: return (buf[3] & 0x01) ? PKT_START : PKT_STOP;
    case 0x01: return (len >= HPSDR_PKT_SIZE) ? PKT_EP2 : PKT_UNKNOWN;
    default:   return PKT_UNKNOWN;
  }
}

// Build a 60-byte discovery reply identifying this device as a HermesLite.
// in_use is set when a session is already active with a different client,
// signalling to the requester that the radio is busy.
static void hpsdr_build_discovery_reply(uint8_t *reply, int in_use) {
  memset(reply, 0, HPSDR_DISCOVERY_REPLY);  // HPSDR_DISCOVERY_REPLY currently defined as 60
  reply[0] = 0xEF;
  reply[1] = 0xFE;
  reply[2] = 0x02;
  reply[3] = in_use ? 0x02 : 0x00;

  // MAC address (fixed, HermesLite-style)
  reply[4] = 0x00;
  reply[5] = 0x1C; reply[6] = 0xC0; reply[7] = 0xA2;
  reply[8] = 0x22; reply[9] = 0x5B;

  reply[10] = 0x06; // Board type: Hermes-Lite
  reply[11] = 0x4A; // Firmware version 74 dec
  reply[19] = 0x01; // MetisVersion
  reply[20] = 0x01; // NumRxs = 1
}

// Unpack one 1032-byte EP2 packet (SDR app → sBitx).
// Extracts the MOX bit, RX/TX frequencies from the C&C round-robin, and up
// to 126 TX IQ sample pairs (16-bit big-endian, normalised to ±1.0).
// Returns the number of IQ sample pairs placed in result->iq[].
static int hpsdr_unpack_ep2(const uint8_t *buf, int len, hpsdr_ep2_result_t *result) {
  if (!buf || len < HPSDR_PKT_SIZE) return 0;

  result->mox       = 0;
  result->freq      = 0;
  result->tx_freq   = 0;
  result->n_samples = 0;

  const uint8_t *ptr = buf + 8; // skip 8-byte Metis header

  for (int frame = 0; frame < 2; frame++) {
    // Validate USB sync bytes
    if (ptr[0] != 0x7F || ptr[1] != 0x7F || ptr[2] != 0x7F) {
      ptr += 512;
      continue;
    }

    uint8_t c0   = ptr[3];
    uint8_t addr = (c0 >> 1) & 0x7F; // C&C round-robin slot index
    int     mox  = c0 & 0x01;        // MOX/PTT bit (Section 8.3)

    // OR the MOX bit across both frames (Section 5.5)
    result->mox |= mox;

    // 19 standard slots decoded; HL2 extension slots listed but not used
    switch (addr) {
      case 0x00: { // General Settings: sample rate in C1 bits 1:0
        uint8_t rate_bits = ptr[4] & 0x03;
        if      (rate_bits == 0) hpsdr_sample_rate =  48000;
        else if (rate_bits == 1) hpsdr_sample_rate =  96000;
        else if (rate_bits == 2) hpsdr_sample_rate = 192000;
        else if (rate_bits == 3) hpsdr_sample_rate = 384000;
        break;
      }
      case 0x01: // TX VFO frequency
        result->tx_freq = ((uint32_t)ptr[4] << 24) | ((uint32_t)ptr[5] << 16) |
                          ((uint32_t)ptr[6] <<  8) |  (uint32_t)ptr[7];
        //printf("hpsdr EP2 addr 0x01 tx_freq = %u\n", result->tx_freq);
        break;
      case 0x02: // RX1 (DDC0) frequency
        result->freq    = ((uint32_t)ptr[4] << 24) | ((uint32_t)ptr[5] << 16) |
                          ((uint32_t)ptr[6] <<  8) |  (uint32_t)ptr[7];
        //printf("hpsdr EP2 addr 0x02 freq    = %u\n", result->freq);
        break;
      case 0x03: // RX2 (DDC1) frequency — not used
      case 0x0E: // ADC assignments & TX step attenuator — not used
      case 0x04: case 0x05: case 0x06: case 0x07: case 0x08: // DDC2-6 — not used
      case 0x09: // Drive level & Alex filters — stub (sBitx filter relays go here)
      case 0x0A: // Preamp & RX step attenuator — not used
      case 0x0B: // CW keyer speed & weight — not used
      case 0x0F: // CW enable & sidetone level — not used
      case 0x10: // CW hang delay & sidetone freq — not used
      case 0x11: // EER PWM settings — not used
      case 0x12: // BPF2 / transverter — not used
      case 0x17: // HL2 extension (C0 masked 0x2E) — not used
      case 0x3A: // HL2 extension (C0 masked 0x74) — not used
      default:
        break;
    }

    // Advance past the 8-byte sync+C&C header to the IQ payload
    ptr += 8;
    // Unpack 63 TX IQ sample pairs per USB frame (Section 8.4).
    // Each group is 8 bytes: [L audio 0-1][R audio 2-3][I 4-5][Q 6-7]
    for (int j = 0; j < 63 && result->n_samples < SAMPLES_PER_PKT; j++) {
      int16_t is = (int16_t)(((uint16_t)ptr[4] << 8) | (uint16_t)ptr[5]);
      int16_t qs = (int16_t)(((uint16_t)ptr[6] << 8) | (uint16_t)ptr[7]);

      result->iq[result->n_samples * 2 + 0] = (float)is / 32768.0f * hpsdr_tx_gain;
      result->iq[result->n_samples * 2 + 1] = (float)qs / 32768.0f * hpsdr_tx_gain;

      ptr += 8;
      result->n_samples++;
    }
  }
  return result->n_samples;
}

// Build and send one 1032-byte EP6 packet (sBitx → SDR app).
// Contains two USB frames of 63 IQ sample pairs (24-bit big-endian) drawn
// from iq_buf, plus C&C bytes reporting current sBitx state via a 5-slot
// round-robin (Section 4.3).
// Called from rx_filter_and_decimate() / hpsdr_send_iq() when iq_buf is full.
static void build_and_send_packet(void) {
  uint8_t pkt[HPSDR_PKT_SIZE];
  memset(pkt, 0, sizeof(pkt));

  // Metis header: EP6 (radio → host)
  pkt[0] = 0xEF; pkt[1] = 0xFE; pkt[2] = 0x01; pkt[3] = 0x06;
  pkt[4] = (tx_seq >> 24) & 0xFF;
  pkt[5] = (tx_seq >> 16) & 0xFF;
  pkt[6] = (tx_seq >>  8) & 0xFF;
  pkt[7] =  tx_seq        & 0xFF;

  uint32_t seq_for_cc = tx_seq++;

  for (int frame = 0; frame < 2; frame++) {
    uint8_t *fp = pkt + 8 + frame * 512;
    fp[0] = 0x7F; fp[1] = 0x7F; fp[2] = 0x7F; // USB sync

    // 5-slot C&C round-robin: advance one slot per frame sent
    int cc_addr = (seq_for_cc * 2 + frame) % 5;

    // C0: slot index in bits 7:3, current PTT/MOX state in bit 0
    fp[3] = (cc_addr << 3) | (in_tx ? 1 : 0);

    // C1–C4: slot payload (Section 4.3)
    switch (cc_addr) {
      case 0: // ADC overload flags & firmware version
        fp[4] = 0x00; // C1: ADC overload (0 = no clip)
        fp[5] = 0x00; // C2: digital inputs
        fp[6] = 0x4A; // C3: firmware version 74
        fp[7] = 0x00; // C4: reserved
        break;
      case 1: // Exciter power (C1-C2) & forward PA power (C3-C4)
        fp[4] = 0x00; fp[5] = 0x00;
        fp[6] = 0x00; fp[7] = 0x00;
        break;
      case 2: // Reverse PA power (C1-C2) & PA voltage / User_ADC0 (C3-C4)
        fp[4] = 0x00; fp[5] = 0x00;
        fp[6] = 0x00; fp[7] = 0x00;
        break;
      case 3: // PA current / User_ADC1 (C1-C2) & supply voltage (C3-C4)
        fp[4] = 0x00; fp[5] = 0x00;
        fp[6] = 0x00; fp[7] = 0x00;
        break;
      case 4: // Additional ADC overload flags: ADC0 (C1), ADC1 (C2), ADC2 (C3)
        fp[4] = 0x00; fp[5] = 0x00;
        fp[6] = 0x00; fp[7] = 0x00;
        break;
    }

    // 63 IQ sample pairs per frame, packed as 24-bit big-endian I then Q,
    // followed by 2 bytes of mic audio (unused, left as zero)
    for (int s = 0; s < 63; s++) {
      int     idx = frame * 63 + s;
      uint8_t *sp = fp + 8 + s * 8;

      int32_t i_val = (int32_t)(iq_buf_i[idx] * 559240.0);
      if (i_val >  8388607)  i_val =  8388607;
      if (i_val < -8388608)  i_val = -8388608;

      int32_t q_val = (int32_t)(iq_buf_q[idx] * 559240.0);
      if (q_val >  8388607)  q_val =  8388607;
      if (q_val < -8388608)  q_val = -8388608;

      sp[0] = (i_val >> 16) & 0xFF;
      sp[1] = (i_val >>  8) & 0xFF;
      sp[2] =  i_val        & 0xFF;
      sp[3] = (q_val >> 16) & 0xFF;
      sp[4] = (q_val >>  8) & 0xFF;
      sp[5] =  q_val        & 0xFF;
      sp[6] = 0; // mic high byte (unused)
      sp[7] = 0; // mic low byte  (unused)
    }
  }

  if (client_active) {
    sendto(hpsdr_sock, pkt, sizeof(pkt), 0,
           (struct sockaddr *)&stream_dest, sizeof(stream_dest));
  }
}

// Reset all TX-related state to a clean idle condition.
// This is the single chokepoint for TX teardown — ensures no stale state
// carries over between sessions or after a crash/disconnect.
// Called on PKT_START, PKT_STOP, and watchdog timeout.
static void reset_all_tx_state(void) {
  flush_tx_ring();
  hpsdr_tx_data_active = 0;
  remote_mox           = 0;
  ep2_last_time_ms     = millis_now();
}

// Top-level packet dispatcher: classify each inbound UDP packet and route it
// to the appropriate handler. EP2 packets are unpacked here, then handed off
// to Section 4 for sBitx state changes.
static void handle_command(uint8_t *buf, int len, struct sockaddr_in *sender) {
  switch (hpsdr_classify(buf, len)) {

  case PKT_DISCOVERY: {
    uint8_t reply[HPSDR_DISCOVERY_REPLY];
    // Report "in use" only if the active session belongs to a different host
    int same = (sender->sin_addr.s_addr == stream_dest.sin_addr.s_addr);
    hpsdr_build_discovery_reply(reply, client_active && !same);
    sendto(hpsdr_sock, reply, sizeof(reply), 0,
           (struct sockaddr *)sender, sizeof(*sender));
    break;
  }

  case PKT_START:
    stream_dest       = *sender;
    tx_seq            = 0;
    iq_buf_count      = 0;
    hpsdr_sample_rate = 48000; // reset to default; client will re-negotiate
    reset_all_tx_state();
    last_rx_freq = 0;
    last_tx_freq = 0;
    client_active = 1;
    printf("hpsdr: streaming STARTED to %s:%d\n",
           inet_ntoa(stream_dest.sin_addr), ntohs(stream_dest.sin_port));
    break;

  case PKT_STOP:
    client_active = 0;
    printf("hpsdr: streaming STOPPED\n");
    if (hpsdr_tx_data_active) {
      hpsdr_tx_data_active = 0;
      if (tr_pending != 2) {
        tr_pending = 2;
        g_idle_add(hpsdr_tr_idle, NULL);
      }
    }
    remote_mox = 0;
    break;

  case PKT_EP2: {
    if (!client_active) break;
    ep2_last_time_ms = millis_now();

    hpsdr_ep2_result_t r;
    hpsdr_unpack_ep2(buf, len, &r);

    // Persist the RX1 frequency across the 19-slot round-robin gap.
    // addr 0x02 is zero in 18 of every 19 frames.
    if (r.freq) last_rx_freq = r.freq;
    if (r.tx_freq) last_tx_freq = r.tx_freq;

    // During RX, follow the spectrum LO continuously.
    // During TX (either remote MOX or physical key), skip — must not override
    // the TX VFO back to the RX LO on every EP2 packet.
    if (!remote_mox && !in_tx)
      apply_freq_from_ep2(last_rx_freq);
    apply_mox_from_ep2(r.mox);

    for (int k = 0; k < r.n_samples; k++)
      tx_upsample_and_push(r.iq[k * 2], r.iq[k * 2 + 1]);
    break;
  }

  }   // switch statement
}

// =============================================================================
// SECTION 4 — sBITX STATE INTEGRATION
// =============================================================================
// translate HPSDR protocol state into sBitx hardware/core
// actions. No packet I/O happens here; this section only drives the sBitx
// externals (remote_execute, tx_on, tx_off) in response to decoded EP2 data.
//
// Thread note: apply_freq_from_ep2() and apply_mox_from_ep2() run on the
// poll thread. The T/R switch calls (tx_on/tx_off) must run on the GTK main
// thread, so they are deferred via g_idle_add(hpsdr_tr_idle).
//
// Future additions: gain settings, band info, antenna selection, etc.

// Apply a decoded RX or TX VFO frequency to the sBitx, but only when it
// differs from the current sBitx tuned frequency.
static void apply_freq_from_ep2(uint32_t freq) {
  if (freq > 0 && freq != (uint32_t)freq_hdr) {
    //printf("hpsdr: remote set freq %u Hz\n", freq);
    char cmd[50];
    snprintf(cmd, sizeof(cmd), "freq %u", freq);
    remote_execute(cmd);
  }
}

// Translate the EP2 MOX bit into a debounced T/R switch action.
// The new state must be stable for 4 consecutive EP2 packets (~10 ms)
// before the transition is committed, preventing glitches from a single
// spurious packet or brief contact bounce.
static void apply_mox_from_ep2(int mox) {
  static int mox_count         = 0;
  static int mox_pending_state = 0;

  if (mox != remote_mox) {
    // Incoming MOX differs from committed state — accumulate stable count
    if (mox != mox_pending_state) {
      // Direction changed before threshold — restart the counter
      mox_pending_state = mox;
      mox_count = 1;
    } else {
      mox_count++;
    }

    if (mox_count >= 4) {
      // Stable for ~4 packets (~10 ms) — commit the transition
      remote_mox = mox_pending_state;
      mox_count  = 0;
      if (remote_mox) {
        // Tune to operator's selected signal before the T/R switch fires
        //if (last_rx_freq) apply_freq_from_ep2(last_rx_freq);
        hpsdr_tx_data_active = 1;
        //printf("hpsdr: MOX ON (debounced)\n");
        if (tr_pending != 1) {
          tr_pending = 1;
          g_idle_add(hpsdr_tr_idle, NULL);
        }
      } else {
        hpsdr_tx_data_active = 0;
        flush_tx_ring();
        // Restore the RX spectrum centre VFO when returning to receive
        if (last_rx_freq) apply_freq_from_ep2(last_rx_freq);
        //printf("hpsdr: MOX OFF (debounced)\n");
        if (tr_pending != 2) {
          tr_pending = 2;
          g_idle_add(hpsdr_tr_idle, NULL);
        }
      }
    }
  } else {
    // MOX matches committed state — reset debounce counters
    mox_count         = 0;
    mox_pending_state = remote_mox;
  }
}

// GTK main-thread callback to execute a pending T/R switch.
// Deferred here because tx_on()/tx_off() must not be called from the poll thread.
// tr_pending values: 0 = nothing, 1 = go TX, 2 = go RX
static gboolean hpsdr_tr_idle(gpointer data) {
  (void)data;
  int action = tr_pending;
  tr_pending = 0;

  if (action == 1 && !in_tx) {
    // addr 0x01 = operator's selected signal ("RX 1" in SDR Console).
    // For SPARK SDR the selected signal is always the spectrum center so
    // last_tx_freq == last_rx_freq; fall back to last_rx_freq if addr 0x01
    // was never received.
    uint32_t tx_freq = last_tx_freq ? last_tx_freq : last_rx_freq;
    //printf("hpsdr_tr_idle: last_rx_freq=%u freq_hdr=%d\n", last_rx_freq, freq_hdr);
    if (tx_freq) {
      char cmd[50];
      snprintf(cmd, sizeof(cmd), "freq %u", tx_freq);
      cmd_exec(cmd);
      //printf("hpsdr_tr_idle: set TX freq to %u Hz\n", tx_freq);
    }
    //printf("hpsdr_tr_idle: switching to TX\n");
    tx_on(TX_SOFT);
  } else if (action == 2) {
    //printf("hpsdr_tr_idle: switching to RX (in_tx=%d)\n", in_tx);
    tx_off();
  }
  return G_SOURCE_REMOVE;
}

// =============================================================================
// SECTION 5 — INITIALIZATION, CONTROL & SHUTDOWN
// =============================================================================
// manage the lifecycle of the HPSDR interface — open/close the
// UDP socket, spawn the poll thread, run the EP2 watchdog, and expose the
// public control API used by sbitx.c.
//
// The poll thread runs hpsdr_poll_thread() which blocks on recvfrom() and
// dispatches every inbound packet through handle_command() (Section 3).
// The watchdog fires every 50 ms on the GTK main thread and forces a return
// to RX if no EP2 has been received for EP2_WATCHDOG_MS while in TX — covering
// crash, network drop, or any other unclean disconnect.

// Poll thread handle — used only within this section
static pthread_t poll_thread;
static int       poll_thread_started = 0;

// Force RX after this many ms of EP2 silence while in TX
#define EP2_WATCHDOG_MS 3500

static gboolean hpsdr_watchdog(gpointer data) {
  (void)data;
  static int last_in_tx = 0;

  if (!running)
    return G_SOURCE_REMOVE;

  // the 'tr_switch' for keying the sbitx while connected to a SDR app
  // using cmd_exec and tx_on
  // Detect physical key/PTT press: in_tx transitioned to 1 without us
  // initiating it via MOX.  Correct the frequency to the SDR app's selected
  // signal now that we're on the GTK main thread where cmd_exec is safe.
  if (client_active && in_tx && !last_in_tx && !remote_mox) {
    // physical key/PTT only — MOX path already set freq in hpsdr_tr_idle
    uint32_t tx_freq = last_tx_freq ? last_tx_freq : last_rx_freq;
    if (tx_freq) {
      char cmd[50];
      snprintf(cmd, sizeof(cmd), "freq %u", tx_freq);
      //printf("hpsdr watchdog: TX detected, last_rx_freq=%u freq_hdr=%d remote_mox=%d\n", last_rx_freq, freq_hdr, remote_mox);
      cmd_exec(cmd);
      //printf("hpsdr watchdog: PTT detected, corrected TX freq to %u Hz\n", tx_freq);
    }
  }
  last_in_tx = in_tx;

  // Existing watchdog: force RX if EP2 goes silent while in TX
  if (client_active && remote_mox && in_tx &&
      (millis_now() - ep2_last_time_ms > EP2_WATCHDOG_MS)) {
    //printf("hpsdr watchdog: no EP2 for >%dms — forcing RX\n", EP2_WATCHDOG_MS);
    reset_all_tx_state();
    tx_off();
  }
  return G_SOURCE_CONTINUE;
}

static void *hpsdr_poll_thread(void *arg) {
  (void)arg;
  uint8_t buf[2048];
  struct sockaddr_in sender;
  socklen_t sender_len;

  while (running) {
    sender_len = sizeof(sender);
    int n = recvfrom(hpsdr_sock, buf, sizeof(buf), 0,
                     (struct sockaddr *)&sender, &sender_len);
    if (n > 0)
      handle_command(buf, n, &sender);
  }
  return NULL;
}

// Open the UDP socket bound to HPSDR_PORT and prepare it for use.
// Returns 0 on success, -1 on failure.
int hpsdr_init(void) {
  hpsdr_sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (hpsdr_sock < 0) return -1;

  int optval = 1;
  setsockopt(hpsdr_sock, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
  setsockopt(hpsdr_sock, SOL_SOCKET, SO_BROADCAST, &optval, sizeof(optval));

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family      = AF_INET;
  addr.sin_port        = htons(HPSDR_PORT);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);

  if (bind(hpsdr_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    close(hpsdr_sock);
    hpsdr_sock = -1;
    return -1;
  }

  // 200 ms receive timeout so the poll thread can check `running` periodically
  struct timeval tv = {.tv_sec = 0, .tv_usec = 200000};
  setsockopt(hpsdr_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  running = 1;
  return 0;
}

// Close the socket and mark the interface as stopped.
// The poll thread will exit at its next recvfrom() timeout.
void hpsdr_stop(void) {
  running       = 0;
  client_active = 0;
  if (hpsdr_sock >= 0) {
    close(hpsdr_sock);
    hpsdr_sock = -1;
  }
  if (poll_thread_started) {
    pthread_join(poll_thread, NULL);   // blocks until poll thread exits cleanly
    poll_thread_started = 0;           // reset so hpsdr_init/poll can restart safely
  }
}

// Returns 1 if a client session is currently active.
int hpsdr_is_connected(void) {
  return client_active;
}

// Start the poll thread and watchdog timer on the first call after hpsdr_init().
// Safe to call repeatedly — the thread and timer are created only once.
void hpsdr_poll(void) {
  if (!poll_thread_started && running) {
    pthread_create(&poll_thread, NULL, hpsdr_poll_thread, NULL);
    g_timeout_add(50, hpsdr_watchdog, NULL);  // 50 ms timer
    poll_thread_started = 1;
  }
}
