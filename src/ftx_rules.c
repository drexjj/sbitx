// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025 Shawn Rutledge K7IHZ / LB2JK <s@ecloud.org>

#include <sqlite3.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "ftx_rules.h"
#include "sdr_ui.h"

#define PCRE2_CODE_UNIT_WIDTH 8
#include <pcre2.h>

#define MAX_RULES 128

static sqlite3* db = NULL;

static ftx_rule rules[MAX_RULES]; // ought to be enough ;-)
static int rules_count = 0;
static pcre2_match_data *match_data = NULL;

bool db_open()
{
	if (db)
		return true; // already done

	char db_path[PATH_MAX];
	snprintf(db_path, sizeof(db_path), "%s/sbitx/data/sbitx.db", getenv("HOME"));

	int rc = sqlite3_open(db_path, &db);
	if (rc == SQLITE_OK)
		return true;

	fprintf(stderr, "Failed to open database.\n");
	return false;
}

int load_ftx_rules()
{
	if (!db_open())
		return -1;

	int count = 0;
	sqlite3_stmt *stmt;
	sqlite3_prepare_v2(db,
		"select id, field, regex, min, max, cq_priority_adj, ans_priority_adj from ftx_rules;",
		-1, &stmt, NULL);

	for (count = 0; count < MAX_RULES && sqlite3_step(stmt) == SQLITE_ROW; ++count) {
		const int rule_id = sqlite3_column_int(stmt, 0);
		if (rule_id > 127) {
			printf("skipping high-numbered rule %d\n", rule_id);
			continue;
		}
		ftx_rule rule;
		memset(&rule, 0, sizeof(rule));
		rule.id = rule_id;
		const char *field_name = sqlite3_column_text(stmt, 1);
		const unsigned char *regex_s = sqlite3_column_text(stmt, 2);
		rule.min_value = sqlite3_column_int(stmt, 3);
		rule.max_value = sqlite3_column_int(stmt, 4);
		rule.cq_resp_pri_adj = sqlite3_column_int(stmt, 5);
		rule.ans_pri_adj = sqlite3_column_int(stmt, 6);

		if (!strncmp(field_name, "call", 4))
			rule.field = RULE_FIELD_CALLSIGN;
		else if (!strncmp(field_name, "cq_token", 8))
			rule.field = RULE_FIELD_CQ_TOKEN;
		else if (!strncmp(field_name, "country", 7))
			rule.field = RULE_FIELD_COUNTRY;
		else if (!strncmp(field_name, "grid", 4))
			rule.field = RULE_FIELD_GRID;
		else if (!strncmp(field_name, "snr", 3))
			rule.field = RULE_FIELD_SNR;
		else if (!strncmp(field_name, "distance", 8))
			rule.field = RULE_FIELD_DISTANCE;
		else if (!strncmp(field_name, "bearing", 7) || !strncmp(field_name, "azimuth", 7))
			rule.field = RULE_FIELD_AZIMUTH;

		int regex_len = regex_s ? strlen(regex_s) : 0;

		// https://pcre2project.github.io/pcre2/doc/pcre2_compile/
		if (regex_len > 0) {
			int errcode = 0;
			PCRE2_SIZE erroffset = -1;
			rule.regex = pcre2_compile_8(regex_s, regex_len, 0, &errcode, &erroffset, NULL);
			if (!rule.regex) {
				unsigned char buf[1024];
				int buflen = pcre2_get_error_message_8(errcode, buf, sizeof(buf));
				printf("error compiling regex '%s' @ offset %uld: %s\n", regex_s, erroffset, buf);
			} else if (!match_data) {
				// Rule regular expressions are not allowed to have captures (not explicitly enforced yet).
				// So we assume the same pcre2_match_data can be reused for every match.s
				match_data = pcre2_match_data_create_from_pattern(rule.regex, NULL);
			}
		}
		if (rule.cq_resp_pri_adj || rule.ans_pri_adj) {
			rules[count] = rule;
		// 	printf("rule i %d ID %d: field_s %d regex '%s' min %d max %d pri adj %d\n",
		// 			count, rule.id, rule.field, regex_s, rule.min_value, rule.max_value, rule.cq_resp_pri_adj);
		// } else {
		// 	printf("skipping disabled rule %d\n", rule.id);
		}
	}
	if (count == MAX_RULES)
		printf("only %d rules are allowed: ignoring the rest", MAX_RULES);
	sqlite3_finalize(stmt);
	rules_count = count;
	return count;
}

// cleanup at shutdown, or when rules need to be re-loaded
void clear_ftx_rules()
{
	for (int i = 0; i < rules_count; ++i) {
		if (rules[i].regex)
			pcre2_code_free(rules[i].regex);
	}
	printf("clearing %d rules %d bytes\n", rules_count, sizeof(rules));
	memset(rules, 0, sizeof(rules));
	rules_count = 0;
	if (match_data)
		pcre2_match_data_free(match_data);
	match_data = NULL;
}

int ftx_rules_count()
{
	return rules_count;
}

int ftx_priority(const char *text, int text_len, const text_span_semantic *spans, int sem_count, bool *is_to_me)
{
	if (!rules_count)
		return 0;

	const char *cq_token = NULL, *to = NULL, *to_me = NULL, *de = NULL, *country = NULL, *grid = NULL;
	int cq_token_len = 0, to_len = 0, de_len = 0, country_len = 0, grid_len = 0;
	char buf[8];
	int snr = 999, distance = 0, azimuth = -1;

	// Find the fields we're interested in
	for (int s = 1; s < sem_count; ++s) {
		if (!spans[s].length)
			break; // a zero-length span marks the end of the spans array
		switch (spans[s].semantic) {
			case STYLE_SNR:
				if (extract_single_semantic(text, text_len, spans[s], buf, sizeof(buf)) >= 0)
					snr = atoi(buf);
				break;
			case STYLE_DISTANCE:
				if (extract_single_semantic(text, text_len, spans[s], buf, sizeof(buf)) >= 0)
					distance = atoi(buf);
				break;
			case STYLE_AZIMUTH:
				if (extract_single_semantic(text, text_len, spans[s], buf, sizeof(buf)) >= 0)
					azimuth = atoi(buf);
				break;
			case STYLE_GRID:
			case STYLE_EXISTING_GRID:
				grid = text + spans[s].start_column;
				grid_len = spans[s].length;
				break;
			case STYLE_COUNTRY:
				country = text + spans[s].start_column;
				country_len = spans[s].length;
				break;
			case STYLE_FT8_RX:
				if (!strncmp(text + spans[s].start_column, "CQ", 2) && spans[s].length > 3) {
					cq_token = text + spans[s].start_column + 3;
					cq_token_len = spans[s].length - 3;
				}
				break;
			case STYLE_MYCALL:
				to_me = text + spans[s].start_column;
				to_len = spans[s].length;
				break;
			case STYLE_CALLEE:
				to = text + spans[s].start_column;
				to_len = spans[s].length;
				break;
			case STYLE_CALLER:
			case STYLE_RECENT_CALLER:
				de = text + spans[s].start_column;
				de_len = spans[s].length;
				break;
		}
	}

	int ret = 0; // default priority if no rules match
	for (int i = 0; i < rules_count; ++i) {
		ftx_rule rule = rules[i];
		const char *field_s = NULL;
		int field_len = 0;
		switch(rule.field) {
		// regex rules: just set field_s and field_len; we'll check below if there's a valid regex
		case RULE_FIELD_CALLSIGN:
			field_s = de;
			field_len = de_len;
			break;
		case RULE_FIELD_CQ_TOKEN:
			field_s = cq_token;
			field_len = cq_token_len;
			break;
		case RULE_FIELD_COUNTRY:
			field_s = country;
			field_len = country_len;
			break;
		case RULE_FIELD_GRID:
			field_s = grid;
			field_len = grid_len;
			break;
		// numeric rules: adjust priority if it applies
		case RULE_FIELD_SNR:
			if (snr < 999 && snr >= rule.min_value && snr <= rule.max_value)
				ret += to_me ? rule.ans_pri_adj : rule.cq_resp_pri_adj;
			break;
		case RULE_FIELD_DISTANCE:
			// printf("checking distance rule: %d min %d max %d (btw az %d)\n", distance, rule.min_value, rule.max_value, azimuth);
			// max is a 16-bit signed number: not quite big enough for Earth's circumference; so
			// if rule.max_value == -1 it means there is no maximum
			// (there's probably no reason you'd want a max anyway)
			if (distance > 0 && distance >= rule.min_value && (rule.max_value < 0 || distance <= rule.max_value))
				ret += to_me ? rule.ans_pri_adj : rule.cq_resp_pri_adj;
			break;
		case RULE_FIELD_AZIMUTH:
			if (azimuth >= 0 && azimuth >= rule.min_value && azimuth <= rule.max_value)
				ret += to_me ? rule.ans_pri_adj : rule.cq_resp_pri_adj;
			break;
		}
		// If the rule applies to a field that was found, and there is a regex,
		// try a match to see whether we need to adjust priority
		if (rule.regex && field_s && field_len) {
			// printf("checking '%s' against rule %d: field %d prioadj %d\n",
			// 		field_s, rule.id, rule.field, to_me ? rule.ans_pri_adj : rule.cq_resp_pri_adj);
			int m = pcre2_match_8(rule.regex, field_s, field_len, 0, 0, match_data, NULL);
			if (m > 0) {
				// printf("%s matched rule %d: %d\n", field_s, rule.id, m);
				ret += to_me ? rule.ans_pri_adj : rule.cq_resp_pri_adj;
			}
		}
	} // loop over rules

	if (is_to_me)
		*is_to_me = to_me;
	return ret;
}
