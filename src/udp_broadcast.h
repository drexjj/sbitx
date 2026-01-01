#ifndef UDP_BROADCAST_H
#define UDP_BROADCAST_H

#include <stdint.h>
#include <stdbool.h>

/**
 * WSJT-X UDP Protocol Broadcasting
 *
 * This module implements the WSJT-X UDP protocol for broadcasting
 * FT4/8 decoded messages, transmitted messages, and status updates
 * to external applications like Gridtracker.
 *
 * Protocol specification:
 * https://sourceforge.net/p/wsjt/wsjtx/ci/master/tree/Network/NetworkMessage.hpp
 */

/**
 * Initialize the WSJT-X UDP broadcast socket
 * Returns 0 on success, -1 on error
 */
int udp_broadcast_init(void);

/**
 * Close the WSJT-X UDP broadcast socket
 */
void udp_broadcast_close(void);

/**
 * Send a Heartbeat message (Type 0)
 * Should be called on startup and periodically (every 15 seconds)
 * Returns 0 on success, -1 on error
 */
int udp_broadcast_heartbeat(void);

/**
 * Send a Status message (Type 1)
 * Called when radio state changes (frequency, mode, TX/RX, etc.)
 *
 * @param frequency Dial frequency in Hz
 * @param mode Mode string ("FT8" or "FT4")
 * @param dx_call DX callsign being worked (or "")
 * @param report Signal report (or "")
 * @param tx_enabled Whether TX is enabled
 * @param transmitting Whether currently transmitting
 * @param decoding Whether currently decoding
 * @param rx_df RX delta frequency in Hz
 * @param tx_df TX delta frequency in Hz
 * @param de_call Own callsign
 * @param de_grid Own grid square
 * @param dx_grid DX grid square (or "")
 */
int udp_broadcast_status(
    uint64_t frequency,
    const char *mode,
    const char *dx_call,
    const char *report,
    bool tx_enabled,
    bool transmitting,
    bool decoding,
    uint32_t rx_df,
    uint32_t tx_df,
    const char *de_call,
    const char *de_grid,
    const char *dx_grid
);

/**
 * Send a Decode message (Type 2)
 * Called for each decoded FT4/8 message
 *
 * @param time_ms Time in milliseconds since midnight
 * @param snr Signal-to-noise ratio in dB
 * @param delta_time Delta time in seconds (usually 0.0)
 * @param delta_freq Delta frequency in Hz
 * @param mode Mode string ("~" for FT8, "+" for FT4)
 * @param message The decoded message text
 * @param low_confidence Whether decode has low confidence
 * @param off_air Whether this is a transmitted message (not received)
 */
int udp_broadcast_decode(
    uint32_t time_ms,
    int32_t snr,
    double delta_time,
    uint32_t delta_freq,
    const char *mode,
    const char *message,
    bool low_confidence,
    bool off_air
);

/**
 * Helper: Convert HH:MM:SS timestamp to milliseconds since midnight
 *
 * @param timestamp String in format "HH:MM:SS" or "HHMMSS"
 * @return Milliseconds since midnight
 */
uint32_t udp_timestamp_to_ms(const char *timestamp);

/**
 * Send a Status message using current radio state from fields
 * This is a convenience function that reads all necessary values
 * from the sbitx field system and sends a Status message.
 * Returns 0 on success, -1 on error
 */
int udp_broadcast_status_auto(void);

#endif /* UDP_BROADCAST_H */
