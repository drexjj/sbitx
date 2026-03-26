/*
 * squelch.c — RF squelch gate for sBitx
 *
 * HOW IT FITS INTO THE SIGNAL PATH
 * ---------------------------------
 *   sbitx.c: sound_process_ssb()
 *     ...
 *     agc2(r);                           // STEP 8 – sets r->signal_avg
 *     squelch_update(r->signal_avg);     // <<< new call (after AGC)
 *
 *     // STEP 9 – gate the speaker output
 *     for (i = 0; i < MAX_BINS / 2; i++) {
 *         sample = cimag(r->fft_time[i + (MAX_BINS / 2)]);
 *         output_speaker[i] = squelch_is_open() ? sample : 0;
 *         output_tx[i] = 0;
 *     }
 *
 * THRESHOLD MAPPING
 * -----------------
 * signal_avg is a smoothed peak magnitude of the FFT time-domain bins
 * (see agc2() in sbitx.c).  Typical noise-floor values are in the
 * low thousands; a strong S9 signal reaches the high millions.
 *
 * squelch_level maps to an open threshold via a base-10 exponential:
 *
 *   level  0 : always open (disabled)
 *   level  1 : threshold ≈    1,000  (opens on very light signals)
 *   level  5 : threshold ≈   10,000
 *   level 10 : threshold ≈  316,000
 *   level 15 : threshold ≈    3.2 M
 *   level 20 : threshold ≈   10 M   (strong signals only)
 *
 * A closing hysteresis of 50 % (threshold / 2) prevents rapid chatter
 * at the squelch edge.  A hang timer (SQUELCH_HANG_TICKS blocks after
 * the signal drops below the close threshold) gives audio a clean
 * tail before muting, typical of radio squelch circuits.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "squelch.h"
#include "sdr_ui.h"   /* write_console(), field_set(), STYLE_LOG */

/* ------------------------------------------------------------------
 * Configuration
 * ------------------------------------------------------------------ */

/* Maximum user-settable level. */
#define SQUELCH_LEVEL_MAX  20

/*
 * Hang timer length in DSP blocks.
 * At 96 kHz / (MAX_BINS/2) = 96000/1024 ≈ 94 blocks/sec,
 * 20 ticks ≈ ~210 ms of audio tail after the signal fades.
 */
#define SQUELCH_HANG_TICKS 20

/* ------------------------------------------------------------------
 * Public state
 * ------------------------------------------------------------------ */
int squelch_on    = 0;   /* feature enabled flag  */
int squelch_level = 0;   /* 0–SQUELCH_LEVEL_MAX   */

/* ------------------------------------------------------------------
 * Private state
 * ------------------------------------------------------------------ */
static int    squelch_gate_open = 1;   /* current gate state           */
static int    squelch_hang_ctr  = 0;   /* hang countdown (blocks)      */

/* ------------------------------------------------------------------
 * Helpers
 * ------------------------------------------------------------------ */

/*
 * Map a user squelch_level (1–20) to an opening signal threshold.
 * Returns 0.0 when level == 0 (always open).
 *
 * Formula: 10 ^ (3 + level * 0.35)
 *   level  1 → 10^3.35 ≈     2,239
 *   level  5 → 10^4.75 ≈    56,234
 *   level 10 → 10^6.5  ≈ 3,162,278
 *   level 20 → 10^10   =  1.0e10   (effectively closed)
 */
static double level_to_open_threshold(int level)
{
    if (level <= 0)
        return 0.0;
    if (level > SQUELCH_LEVEL_MAX)
        level = SQUELCH_LEVEL_MAX;
    return pow(10.0, 3.0 + level * 0.35);
}

/* ------------------------------------------------------------------
 * Public functions
 * ------------------------------------------------------------------ */

void init_squelch(void)
{
    squelch_on       = 0;
    squelch_level    = 0;
    squelch_gate_open = 1;
    squelch_hang_ctr  = 0;
}

int squelch_update(double signal_avg)
{
    /* If squelch feature is off, the gate is always open. */
    if (!squelch_on || squelch_level == 0) {
        squelch_gate_open = 1;
        squelch_hang_ctr  = 0;
        return 1;
    }

    double open_threshold  = level_to_open_threshold(squelch_level);
    double close_threshold = open_threshold * 0.5; /* 50 % hysteresis */

    if (!squelch_gate_open) {
        /* Gate is currently CLOSED — open it if signal is strong enough. */
        if (signal_avg >= open_threshold) {
            squelch_gate_open = 1;
            squelch_hang_ctr  = SQUELCH_HANG_TICKS;
        }
    } else {
        /* Gate is currently OPEN. */
        if (signal_avg >= close_threshold) {
            /* Signal is above the close threshold — reset the hang timer. */
            squelch_hang_ctr = SQUELCH_HANG_TICKS;
        } else {
            /* Signal has dropped below the close threshold. */
            if (squelch_hang_ctr > 0) {
                /* Still within the hang window; stay open and count down. */
                squelch_hang_ctr--;
            } else {
                /* Hang timer expired — close the gate. */
                squelch_gate_open = 0;
            }
        }
    }

    return squelch_gate_open;
}

int squelch_is_open(void)
{
    /* Always let audio through when the feature is disabled. */
    if (!squelch_on || squelch_level == 0)
        return 1;
    return squelch_gate_open;
}

void squelch_toggle(void)
{
    squelch_on = !squelch_on;

    /* Reset gate state so the first block after enabling starts clean. */
    squelch_gate_open = 1;
    squelch_hang_ctr  = 0;

    /* Update the "#squelch" toggle field in the UI. */
    field_set("#squelch", squelch_on ? "ON" : "OFF");

    /* Report to console. */
    char msg[64];
    snprintf(msg, sizeof(msg),
             "\n *Squelch %s (level %d)\n",
             squelch_on ? "ON" : "OFF", squelch_level);
    write_console(STYLE_LOG, msg);
}

void squelch_set_level(int level)
{
    if (level < 0)
        level = 0;
    if (level > SQUELCH_LEVEL_MAX)
        level = SQUELCH_LEVEL_MAX;

    squelch_level = level;

    /* Reset gate so a new threshold takes effect immediately. */
    squelch_gate_open = 1;
    squelch_hang_ctr  = 0;
}
