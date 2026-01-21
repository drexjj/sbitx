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

/*!
	Load all the FT8/FT4 CQRESP/ANS rules from the ftx_rules database table into memory.

	Returns the number of rules loaded, 0 if none, -1 if there is no ftx_rules table.
 */
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
		rule.field = ftx_rule_field_from_name(field_name);

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
	// printf("clearing %d rules %d bytes\n", rules_count, sizeof(rules));
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

/*!
	Returns the priority (score) of an FT8/FT4 message \a text with semantic \a spans.
	If \a is_to_me is given, it will be set to \c true if the message is addressed to my callsign,
	as indicated by one of the \a spans having STYLE_MYCALL.
*/
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

/*!
	Adds a new rule for the given \a field to test against the given regular expression \a regex.
	This is for string fields: callsign, CQ token (such as DX, SOTA, SA, JP etc.), abbreviated
	country name, or grid square. Also give the priority adjustments you'd like to make:
	for example \a cq_resp_pri_adj +1 to prefer to answer CQs that match \a regex, and
	\a ans_pri_adj +1 to prefer answering incoming messages that match, in the rare case
	that you have multiple simultaneous QSOs. You can also give negative priorities:
	for example if \a ans_pri_adj is sufficiently negative to override the other rules,
	the code will never auto-answer matching messages even though they are addressed to
	your callsign.

	For example if you need to get a contact in Malta, you can add a rule:
	```
	ftx_add_regex_rule("+ Malta", RULE_FIELD_COUNTRY, "Malta", +3, +1)
	```
	Priority adjustments must be within the range +/- 127 but should generally
	be much smaller (single digits).

	Returns the ID of the new rule, or -1 on error.
 */
int ftx_add_regex_rule(const char *desc, ftx_rules_field field, const char *regex, int8_t cq_resp_pri_adj, int8_t ans_pri_adj)
{
	if (!db_open())
		return -1;

	const char *field_s = NULL;
	switch (field) {
		case RULE_FIELD_CALLSIGN: field_s = "call"; break;
		case RULE_FIELD_CQ_TOKEN: field_s = "cq_token"; break;
		case RULE_FIELD_COUNTRY:  field_s = "country"; break;
		case RULE_FIELD_GRID:     field_s = "grid"; break;
		default:
			fprintf(stderr, "ftx_add_regex_rule: unsupported field %d\n", field);
			return -1;
	}

	/*	Instead of letting SQLite auto-assign an ever-increasing id, pick the lowest
		available positive id. Use a SELECT subquery so we can keep the existing
		bind parameter layout (desc, field, regex, cq, ans).
		The subquery finds the smallest gap by looking for t1.id+1 values that do
		not exist in the table. If the table is empty, it falls back to 1. */
	const char *sql = "insert into ftx_rules (id, description, field, regex, min, max, cq_priority_adj, ans_priority_adj) "
		"select COALESCE((SELECT t1.id+1 FROM ftx_rules t1 LEFT JOIN ftx_rules t2 ON t1.id+1 = t2.id "
		"WHERE t2.id IS NULL ORDER BY t1.id LIMIT 1), 1), ?, ?, ?, NULL, NULL, ?, ?;";
	sqlite3_stmt *stmt = NULL;
	int rc = sqlite3_prepare_v2(db, sql, -1, &stmt, NULL);
	if (rc != SQLITE_OK) {
		fprintf(stderr, "ftx_add_regex_rule: prepare failed: %s\n", sqlite3_errmsg(db));
		return -1;
	}

	sqlite3_bind_text(stmt, 1, desc, -1, SQLITE_STATIC);
	sqlite3_bind_text(stmt, 2, field_s, -1, SQLITE_STATIC);
	sqlite3_bind_text(stmt, 3, regex, -1, SQLITE_TRANSIENT);
	sqlite3_bind_int(stmt, 4, cq_resp_pri_adj);
	sqlite3_bind_int(stmt, 5, ans_pri_adj);

	rc = sqlite3_step(stmt);
	if (rc != SQLITE_DONE) {
		fprintf(stderr, "ftx_add_regex_rule: insert failed: %s\n", sqlite3_errmsg(db));
		sqlite3_finalize(stmt);
		return -1;
	}

	sqlite3_finalize(stmt);

	/* reload rules so in-memory set reflects DB */
	clear_ftx_rules();
	load_ftx_rules();

	return (int)sqlite3_last_insert_rowid(db);
}

/*!
	Adds a new rule for the given \a field to be applied when the numeric value extracted
	from the incoming FT8/FT4 message is greater than or equal to \a min_value and
	less than or equal to \a max_value.

	If \a max_value is -1, it means there is no upper limit: e.g. to prioritize DX you'd
	make a rule with \a field RULE_FIELD_DISTANCE, \a min_value in km and max_value of -1
	with the priority adjustment you'd like to make, for example \a cq_resp_pri_adj +1
	to prefer to answer CQs with higher distance, and \a ans_pri_adj +1 to prefer
	answering incoming messages from higher distance if you have multiple simultaneous QSOs.

	Priority adjustments must be within the range +/- 127 but should generally
	be much smaller (single digits).

	Returns the ID of the new rule, or -1 on error.
 */
int ftx_add_numeric_rule(const char *desc, ftx_rules_field field, int16_t min_value, int16_t max_value,
		int8_t cq_resp_pri_adj, int8_t ans_pri_adj)
{
	if (!db_open())
		return -1;

	const char *field_s = NULL;
	switch (field) {
		case RULE_FIELD_SNR:      field_s = "snr"; break;
		case RULE_FIELD_DISTANCE: field_s = "distance"; break;
		case RULE_FIELD_AZIMUTH:  field_s = "azimuth"; break;
		default:
			fprintf(stderr, "ftx_add_numeric_rule: unsupported field %d\n", field);
			return -1;
	}

	/*	Instead of letting SQLite auto-assign an ever-increasing id, pick the lowest
		available positive id. Use a SELECT subquery so we can keep the existing
		bind parameter layout (desc, field, regex, cq, ans).
		The subquery finds the smallest gap by looking for t1.id+1 values that do
		not exist in the table. If the table is empty, it falls back to 1. */
	const char *sql = "insert into ftx_rules (id, description, field, regex, min, max, cq_priority_adj, ans_priority_adj) "
		"select COALESCE((SELECT t1.id+1 FROM ftx_rules t1 LEFT JOIN ftx_rules t2 ON t1.id+1 = t2.id "
		"WHERE t2.id IS NULL ORDER BY t1.id LIMIT 1), 1), ?, ?, NULL, ?, ?, ?, ?;";
	sqlite3_stmt *stmt = NULL;
	int rc = sqlite3_prepare_v2(db, sql, -1, &stmt, NULL);
	if (rc != SQLITE_OK) {
		fprintf(stderr, "ftx_add_numeric_rule: prepare failed: %s\n", sqlite3_errmsg(db));
		return -1;
	}

	sqlite3_bind_text(stmt, 1, desc, -1, SQLITE_STATIC);
	sqlite3_bind_text(stmt, 2, field_s, -1, SQLITE_STATIC);
	sqlite3_bind_int(stmt, 3, min_value);
	sqlite3_bind_int(stmt, 4, max_value);
	sqlite3_bind_int(stmt, 5, cq_resp_pri_adj);
	sqlite3_bind_int(stmt, 6, ans_pri_adj);

	rc = sqlite3_step(stmt);
	if (rc != SQLITE_DONE) {
		fprintf(stderr, "ftx_add_numeric_rule: insert failed: %s\n", sqlite3_errmsg(db));
		sqlite3_finalize(stmt);
		return -1;
	}

	sqlite3_finalize(stmt);

	/* reload rules so in-memory set reflects DB */
	clear_ftx_rules();
	load_ftx_rules();

	return (int)sqlite3_last_insert_rowid(db);
}

bool ftx_rule_update_priorities(int8_t id, int8_t cq_resp_pri_adj, int8_t ans_pri_adj)
{
	if (!db_open())
		return false;

	const char *sql = "update ftx_rules set cq_priority_adj = ?, ans_priority_adj = ? where id = ?;";
	sqlite3_stmt *stmt = NULL;
	int rc = sqlite3_prepare_v2(db, sql, -1, &stmt, NULL);
	if (rc != SQLITE_OK) {
		fprintf(stderr, "ftx_rule_update_priorities: prepare failed: %s\n", sqlite3_errmsg(db));
		return false;
	}

	sqlite3_bind_int(stmt, 1, cq_resp_pri_adj);
	sqlite3_bind_int(stmt, 2, ans_pri_adj);
	sqlite3_bind_int(stmt, 3, id);

	rc = sqlite3_step(stmt);
	if (rc != SQLITE_DONE) {
		fprintf(stderr, "ftx_rule_update_priorities: step failed: %s\n", sqlite3_errmsg(db));
		sqlite3_finalize(stmt);
		return false;
	}

	int changes = sqlite3_changes(db);
	sqlite3_finalize(stmt);

	if (changes > 0) {
		clear_ftx_rules();
		load_ftx_rules();
		return true;
	}
	return false;
}

bool ftx_delete_rule(int8_t id)
{
	if (!db_open())
		return false;

	const char *sql = "delete from ftx_rules where id = ?;";
	sqlite3_stmt *stmt = NULL;
	int rc = sqlite3_prepare_v2(db, sql, -1, &stmt, NULL);
	if (rc != SQLITE_OK) {
		fprintf(stderr, "ftx_delete_rule: prepare failed: %s\n", sqlite3_errmsg(db));
		return false;
	}

	sqlite3_bind_int(stmt, 1, id);

	rc = sqlite3_step(stmt);
	if (rc != SQLITE_DONE) {
		fprintf(stderr, "ftx_delete_rule: step failed: %s\n", sqlite3_errmsg(db));
		sqlite3_finalize(stmt);
		return false;
	}

	int changes = sqlite3_changes(db);
	sqlite3_finalize(stmt);

	if (changes > 0) {
		clear_ftx_rules();
		load_ftx_rules();
		return true;
	}
	return false;
}

const char *ftx_rule_field_name(ftx_rules_field field)
{
	switch (field) {
		case RULE_FIELD_CALLSIGN:  return "call";
		case RULE_FIELD_CQ_TOKEN:  return "cq_token";
		case RULE_FIELD_COUNTRY:   return "country";
		case RULE_FIELD_GRID:      return "grid";
		case RULE_FIELD_SNR:       return "snr";
		case RULE_FIELD_DISTANCE:  return "distance";
		case RULE_FIELD_AZIMUTH:   return "azimuth";
		default:                   return "none";
	}
}

ftx_rules_field ftx_rule_field_from_name(const char *name)
{
	if (!name)
		return RULE_FIELD_NONE;

	/* match same substrings as used elsewhere in this file */
	if (!strncmp(name, "call", 4))
		return RULE_FIELD_CALLSIGN;
	if (!strncmp(name, "cq_token", 8))
		return RULE_FIELD_CQ_TOKEN;
	if (!strncmp(name, "country", 7))
		return RULE_FIELD_COUNTRY;
	if (!strncmp(name, "grid", 4))
		return RULE_FIELD_GRID;
	if (!strncmp(name, "snr", 3))
		return RULE_FIELD_SNR;
	if (!strncmp(name, "distance", 8))
		return RULE_FIELD_DISTANCE;
	/* legacy DB might use 'bearing' or 'azimuth' */
	if (!strncmp(name, "bearing", 7) || !strncmp(name, "azimuth", 7))
		return RULE_FIELD_AZIMUTH;

	return RULE_FIELD_NONE;
}

void *ftx_rule_prepare_query_all()
{
	if (!db_open())
		return NULL;

	const char *sql = "select id, description, field, regex, min, max, cq_priority_adj, ans_priority_adj "
		"from ftx_rules order by id;";
	sqlite3_stmt *stmt = NULL;
	int rc = sqlite3_prepare_v2(db, sql, -1, &stmt, NULL);
	if (rc != SQLITE_OK) {
		fprintf(stderr, "ftx_rule_prepare_query_all: prepare failed: %s\n", sqlite3_errmsg(db));
		return NULL;
	}

	return stmt;
}

int ftx_next_rule(void *query, ftx_rule *rule, char *desc_buf, int desc_size, char *regex_buf, int regex_size)
{
	if (!query || !rule)
		return -1;

	sqlite3_stmt *stmt = (sqlite3_stmt *)query;
	int rc = sqlite3_step(stmt);
	if (rc == SQLITE_DONE)
		return 0;
	if (rc != SQLITE_ROW) {
		fprintf(stderr, "ftx_next_rule: step failed: %s\n", sqlite3_errmsg(db));
		return -1;
	}

	memset(rule, 0, sizeof(*rule));

	/* Columns: id, description, field, regex, min, max, cq_priority_adj, ans_priority_adj */
	int id = sqlite3_column_int(stmt, 0);
	const unsigned char *desc = sqlite3_column_text(stmt, 1);
	const unsigned char *field_name = sqlite3_column_text(stmt, 2);
	const unsigned char *regex = sqlite3_column_text(stmt, 3);
	int minv = sqlite3_column_int(stmt, 4);
	int maxv = sqlite3_column_int(stmt, 5);
	int cq_adj = sqlite3_column_int(stmt, 6);
	int ans_adj = sqlite3_column_int(stmt, 7);

	rule->id = (int8_t)id;
	rule->min_value = (int16_t)minv;
	rule->max_value = (int16_t)maxv;
	rule->cq_resp_pri_adj = (int8_t)cq_adj;
	rule->ans_pri_adj = (int8_t)ans_adj;
	rule->regex = NULL; /* GUI consumer gets the regex in regex_buf */
	rule->field = ftx_rule_field_from_name(field_name);

	/* copy description and regex into provided buffers, if any */
	if (desc && desc_buf && desc_size > 0) {
		strncpy(desc_buf, (const char *)desc, desc_size - 1);
		desc_buf[desc_size - 1] = '\0';
	} else if (desc_buf && desc_size > 0) {
		desc_buf[0] = '\0';
	}

	if (regex && regex_buf && regex_size > 0) {
		strncpy(regex_buf, (const char *)regex, regex_size - 1);
		regex_buf[regex_size - 1] = '\0';
	} else if (regex_buf && regex_size > 0) {
		regex_buf[0] = '\0';
	}

	return 1;
}

void ftx_rule_end_query(void *query)
{
	sqlite3_finalize(query);
}
