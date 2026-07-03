#ifndef SQUELCH_H
#define SQUELCH_H

/*
 * squelch.h — RF squelch gate for sBitx
 *
 * Integrates with the AGC signal_avg path in sbitx.c.
 * The squelch closes the audio path when the received signal
 * falls below the configured threshold, cutting background noise
 * on quiet bands.
 *
 * squelch_level 0   : squelch fully open (disabled), pass-through always
 * squelch_level 1–9 : light squelch, opens on moderate signals
 * squelch_level 10+ : tight squelch, only strong signals break through
 * (practical range is 0–20 for most HF use)
 *
 * Convention matches the rest of the project: squelch_level is the
 * user-facing integer stored in the "SQL" field.
 */

/* ------------------------------------------------------------------
 * Public state (read-only from outside squelch.c)
 * ------------------------------------------------------------------ */

/* 1 = squelch feature enabled, 0 = disabled (pass-through) */
extern int squelch_on;

/* Threshold level 0–20 (stored in "SQL" field). 0 = always open. */
extern int squelch_level;

/* ------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------ */

/* Call once at startup, before the audio thread starts. */
void init_squelch(void);

/*
 * Call once per DSP block (after agc2()) with r->signal_avg.
 * Internally applies hysteresis and a hang timer so brief fades
 * do not chatter the squelch gate.
 * Returns 1 if the gate is open (pass audio), 0 if closed (mute).
 */
int squelch_update(double signal_avg);

/*
 * Returns the current squelch gate state without updating it.
 * Safe to call from the audio output path.
 */
int squelch_is_open(void);

/*
 * Toggle squelch_on between 0 and 1.
 * Writes a status line to the console and updates the SQL field label.
 * Intended to be called from do_squelch_toggle() in sbitx_gtk.c.
 */
void squelch_toggle(void);

/*
 * Set squelch_level (0–20). Clamps out-of-range values.
 * Used by the "SQL" FIELD_NUMBER handler and \sql command.
 */
void squelch_set_level(int level);

#endif /* SQUELCH_H */
