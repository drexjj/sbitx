// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025 Shawn Rutledge K7IHZ / LB2JK <s@ecloud.org>

#ifndef FTX_RULES_H
#define FTX_RULES_H

#include <stdint.h>

#include "sdr_ui.h"

// programmatic names for rules from the ftx_rules table
typedef enum {
	RULE_FIELD_NONE = 0, // invalid field
	RULE_FIELD_CALLSIGN, // regular expression to match "de" callsign
	RULE_FIELD_CQ_TOKEN, // CQ "SOTA" and such
	RULE_FIELD_COUNTRY,  // regular expression to match full name or abbreviated name
	RULE_FIELD_GRID,	 // regular expression
	RULE_FIELD_SNR,      // int min/max, in dB
	RULE_FIELD_DISTANCE, // int min/max, in km
	RULE_FIELD_AZIMUTH,  // int min/max, in degrees from 0 to 360
}  ftx_rules_field;

/*	A representation of a rule from the ftx_rules table
 	for the purpose of applying to incoming messages
    (i.e. it's kept small and efficient).

    This struct takes 128 bits, 2 machine words.
    Good alignment and small size helps make the rule engine efficient.
*/
typedef struct {
	int8_t id;
	int8_t field; // ftx_rules_field enum value
	int8_t cq_resp_pri_adj; // amount to add or subtract from CQRESP priority
	int8_t ans_pri_adj; // amount to add or subtract from priority for answering direct messages
	int16_t min_value; // minimum value for snr, distance, or azimuth
	int16_t max_value; // maximum value for snr, distance, or azimuth
	void *regex; // pcre2_code * return value from pcre2_compile()
} ftx_rule;

/*! Load all rules from the ftx_rules database table.
 	Returns the number found, 0 if none, -1 if there is no ftx_rules table. */
int load_ftx_rules();

void clear_ftx_rules();

int ftx_rules_count();

/*! Returns the priority (score) of an FT8/FT4 message \a text with semantic spans. */
int ftx_priority(const char *text, int text_len, const text_span_semantic *sem, int sem_count, bool *is_to_me);

#endif // FTX_RULES_H
