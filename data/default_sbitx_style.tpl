; sBitx Style Configuration
; This file controls the colors and fonts used in the UI
; Color formats: RGB floats (0.0-1.0), RGB ints (0-255), or hex (#RRGGBB)

[general]
ui_font = Sans
field_font_size = 12

; Color palette definitions
; Format: [color:NAME] then index=N and rgb=r,g,b

[color:COLOR_SELECTED_TEXT]
index = 0
rgb = 1.00, 1.00, 1.00

[color:COLOR_TEXT]
index = 1
rgb = 0.00, 1.00, 1.00

[color:COLOR_TEXT_MUTED]
index = 2
rgb = 0.50, 0.50, 0.50

[color:COLOR_SELECTED_BOX]
index = 3
rgb = 1.00, 1.00, 0.00

[color:COLOR_BACKGROUND]
index = 4
rgb = 0.00, 0.00, 0.00

[color:COLOR_FREQ]
index = 5
rgb = 1.00, 1.00, 0.00

[color:COLOR_LABEL]
index = 6
rgb = 1.00, 0.00, 1.00

[color:SPECTRUM_BACKGROUND]
index = 7
rgb = 0.00, 0.00, 0.00

[color:SPECTRUM_GRID]
index = 8
rgb = 0.10, 0.10, 0.10

[color:SPECTRUM_PLOT]
index = 9
rgb = 1.00, 1.00, 0.00

[color:SPECTRUM_NEEDLE]
index = 10
rgb = 0.20, 0.20, 0.20

[color:COLOR_CONTROL_BOX]
index = 11
rgb = 0.50, 0.50, 0.50

[color:SPECTRUM_BANDWIDTH]
index = 12
rgb = 0.20, 0.20, 0.20

[color:COLOR_RX_PITCH]
index = 13
rgb = 0.00, 1.00, 0.00

[color:SELECTED_LINE]
index = 14
rgb = 0.10, 0.10, 0.20

[color:COLOR_FIELD_SELECTED]
index = 15
rgb = 0.10, 0.10, 0.20

[color:COLOR_TX_PITCH]
index = 16
rgb = 1.00, 0.00, 0.00

[color:COLOR_TOGGLE_ACTIVE]
index = 17
rgb = 0.00, 0.20, 0.00

; Font style definitions
; Format: [font:STYLE_NAME] then index=N and other properties

[font:STYLE_LOG]
index = 0
family = Mono
size = 11
color = 0.70, 0.70, 0.70
weight = normal
slant = normal

[font:STYLE_MYCALL]
index = 1
family = Mono
size = 11
color = 1.00, 0.00, 0.00
weight = normal
slant = normal

[font:STYLE_CALLER]
index = 2
family = Mono
size = 11
color = 0.80, 0.40, 0.00
weight = normal
slant = normal

[font:STYLE_RECENT_CALLER]
index = 3
family = Mono
size = 11
color = 0.00, 0.60, 0.00
weight = normal
slant = normal

[font:STYLE_CALLEE]
index = 4
family = Mono
size = 11
color = 0.00, 0.60, 0.00
weight = normal
slant = normal

[font:STYLE_GRID]
index = 5
family = Mono
size = 11
color = 1.00, 0.80, 0.00
weight = normal
slant = normal

[font:STYLE_EXISTING_GRID]
index = 6
family = Mono
size = 11
color = 0.00, 0.60, 0.00
weight = normal
slant = normal

[font:STYLE_RST]
index = 7
family = Mono
size = 11
color = 0.70, 0.70, 0.70
weight = normal
slant = normal

[font:STYLE_TIME]
index = 8
family = Mono
size = 11
color = 0.00, 0.80, 0.80
weight = normal
slant = normal

[font:STYLE_SNR]
index = 9
family = Mono
size = 11
color = 1.00, 1.00, 1.00
weight = normal
slant = normal

[font:STYLE_FREQ]
index = 10
family = Mono
size = 11
color = 0.00, 0.70, 0.50
weight = normal
slant = normal

[font:STYLE_COUNTRY]
index = 11
family = Mono
size = 11
color = 0.00, 1.00, 0.00
weight = normal
slant = normal

[font:STYLE_DISTANCE]
index = 12
family = Mono
size = 11
color = 1.00, 0.80, 0.00
weight = normal
slant = italic

[font:STYLE_AZIMUTH]
index = 13
family = Mono
size = 11
color = 0.60, 0.40, 0.00
weight = normal
slant = normal

[font:STYLE_FT8_RX]
index = 14
family = Mono
size = 11
color = 0.00, 1.00, 0.00
weight = normal
slant = normal

[font:STYLE_FT8_TX]
index = 15
family = Mono
size = 11
color = 1.00, 0.60, 0.00
weight = normal
slant = normal

[font:STYLE_FT8_QUEUED]
index = 16
family = Mono
size = 11
color = 0.50, 0.50, 0.50
weight = normal
slant = normal

[font:STYLE_FT8_REPLY]
index = 17
family = Mono
size = 11
color = 1.00, 0.60, 0.00
weight = normal
slant = normal

[font:STYLE_CW_RX]
index = 18
family = Mono
size = 11
color = 0.00, 1.00, 0.00
weight = normal
slant = normal

[font:STYLE_CW_TX]
index = 19
family = Mono
size = 11
color = 1.00, 0.60, 0.00
weight = normal
slant = normal

[font:STYLE_FLDIGI_RX]
index = 20
family = Mono
size = 11
color = 0.00, 1.00, 0.00
weight = normal
slant = normal

[font:STYLE_FLDIGI_TX]
index = 21
family = Mono
size = 11
color = 1.00, 0.60, 0.00
weight = normal
slant = normal

[font:STYLE_TELNET]
index = 22
family = Mono
size = 11
color = 0.00, 1.00, 0.00
weight = normal
slant = normal

[font:STYLE_HIGHLIGHT]
index = 23
family = Mono
size = 11
color = 1.00, 1.00, 1.00
weight = normal
slant = normal

[font:STYLE_FIELD_LABEL]
index = 24
family = Mono
size = 14
color = 0.00, 1.00, 1.00
weight = normal
slant = normal

[font:STYLE_FIELD_VALUE]
index = 25
family = Mono
size = 14
color = 1.00, 1.00, 1.00
weight = normal
slant = normal

[font:STYLE_LARGE_FIELD]
index = 26
family = Mono
size = 14
color = 0.00, 1.00, 1.00
weight = normal
slant = normal

[font:STYLE_LARGE_VALUE]
index = 27
family = Arial
size = 24
color = 1.00, 1.00, 1.00
weight = normal
slant = normal

[font:STYLE_SMALL]
index = 28
family = Mono
size = 10
color = 0.00, 1.00, 1.00
weight = normal
slant = normal

[font:STYLE_SMALL_FIELD_VALUE]
index = 29
family = Mono
size = 11
color = 1.00, 1.00, 1.00
weight = normal
slant = normal

[font:STYLE_BLACK]
index = 30
family = Mono
size = 14
color = 0.00, 0.00, 0.00
weight = normal
slant = normal

