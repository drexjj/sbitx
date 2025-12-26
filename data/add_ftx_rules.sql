
create table ftx_rules (
	id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
	description TEXT,
	field TEXT,
	regex TEXT,
	min INTEGER,
	max INTEGER,
	cq_priority_adj INTEGER,
	ans_priority_adj INTEGER
);

-- possible fields: cq_token call country grid distance bearing azimuth snr

-- We like to chase SOTA/POTA etc, but not necessarily "CQ SA", "CQ DX", "CQ JP" etc.
-- You can add more positive priorities for those cases for which your station qualifies.
insert into ftx_rules (description, field, regex, cq_priority_adj)
	values("+ CQ xOTA", "cq_token", "^.OTA", +3);
insert into ftx_rules (description, field, regex, cq_priority_adj)
	values("- CQ xx", "cq_token", "[A-Z][A-Z]", -1);
insert into ftx_rules (description, field, regex, cq_priority_adj, ans_priority_adj)
	values("+ /QRP callsign", "call", "/QRP$", +1, +1);
insert into ftx_rules (description, field, regex, cq_priority_adj, ans_priority_adj)
	values("+ /P callsign", "call", ".*/P", +1, +1);
-- For a distance rule, if max is -1 it means there is no upper limit.
-- These are cumulative: if distance > 3000, priority += 2, because both rules apply.
insert into ftx_rules (description, field, min, max, cq_priority_adj, ans_priority_adj)
	values("+ DX > 1500 km", "distance", 1500, -1, +1, +1);
insert into ftx_rules (description, field, min, max, cq_priority_adj, ans_priority_adj)
	values("+ DX > 3000 km", "distance", 3000, -1, +1, +1);
-- Ignore most CQs, but "pounce" when it's good DX:
-- you just have to reduce priority to less than 0 for anything that isn't DX.
-- Change cq_priority_adj to -3 or so.
insert into ftx_rules (description, field, min, max, cq_priority_adj)
	values("- non-DX < 3000 km", "distance", 0, 3000, 0);
