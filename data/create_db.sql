create table logbook (
	id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
	mode TEXT,
	freq TEXT,
	qso_date TEXT,
	qso_time TEXT,
	callsign_sent TEXT,
	rst_sent TEXT,
	exch_sent TEXT DEFAULT "",
	callsign_recv TEXT,
	rst_recv TEXT,
	exch_recv TEXT DEFAULT "",
	tx_id	TEXT DEFAULT "",
	tx_power TEXT DEFAULT "",
	vswr TEXT DEFAULT "",
	xota TEXT DEFAULT "",
	xota_loc TEXT DEFAULT "",
	comments TEXT DEFAULT ""
);

CREATE INDEX callIx ON logbook(callsign_recv);
CREATE INDEX gridIx ON logbook(exch_recv);

create table ftx_rules (
	id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
	description TEXT,
	field TEXT,
	regex TEXT,
	min INTEGER,
	max INTEGER,
	priority_adj INTEGER
);

-- possible fields: cq_token call country grid distance bearing azimuth snr

-- We like to chase SOTA/POTA etc, but not necessarily "CQ SA", "CQ DX", "CQ JP" etc.
-- You can add more positive priorities for those cases for which your station qualifies.
insert into ftx_rules (description, field, regex, priority_adj)
	values("prioritize CQ xOTA", "cq_token", "^.OTA", +3);
insert into ftx_rules (description, field, regex, priority_adj)
	values("de-prioritize CQ xx", "cq_token", "[A-Z][A-Z]", -1);
insert into ftx_rules (description, field, regex, priority_adj)
	values("prioritize /QRP callsign", "call", "/QRP$", +1);
insert into ftx_rules (description, field, regex, priority_adj)
	values("prioritize /P callsign", "call", ".*/P", +1);
insert into ftx_rules (description, field, max, priority_adj)
	values("de-prioritize not-recently-heard", "age", 180, -1);
-- For a distance rule, if max is -1 it means there is no max.
-- These are cumulative: the longer the distance, priority keeps being added.
insert into ftx_rules (description, field, min, max, priority_adj)
	values("prioritize DX", "distance", 1400, -1, +1);
insert into ftx_rules (description, field, min, max, priority_adj)
	values("prioritize DX", "distance", 2000, -1, +1);
insert into ftx_rules (description, field, min, max, priority_adj)
	values("prioritize DX", "distance", 2500, -1, +1);
insert into ftx_rules (description, field, min, max, priority_adj)
	values("prioritize DX", "distance", 3000, -1, +1);
