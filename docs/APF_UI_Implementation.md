# APF UI Controls Implementation

## Overview

This document describes the UI controls implementation for the Audio Peak Filter (APF) feature in sbitx Menu 1. The APF backend was already present in the codebase; this implementation surfaces those controls in the user interface.

## UI Layout

The APF controls are positioned in Menu 1, to the right of the TUNDUR item:

```
Menu 1 Line 1: SET | TXEQ | RXEQ | NOTCH | ANR | COMP | TXMON | TNDUR | APF | ... | ePTT
Menu 1 Line 2: WEB | EQSET | NFREQ | BNDWTH | DSP | BFO | VFOLK | TNPWR | GAIN | WIDTH
```

### Control Specifications

| Control | Position | Type | Label | Range | Default |
|---------|----------|------|-------|-------|---------|
| APF Toggle | Line 1, X=550 | FIELD_TOGGLE | "APF" | ON/OFF | OFF |
| Gain Control | Line 2, X=550 | FIELD_NUMBER | "GAIN" | 0-20 dB | 6 |
| Width Control | Line 2, X=600 | FIELD_NUMBER | "WIDTH" | 10-500 | 100 |

## Implementation Details

### Files Modified

1. **src/sbitx_gtk.c** - Primary UI implementation
2. **data/default_settings.ini** - Default configuration values

### Code Locations

#### Field Definitions
**File:** `src/sbitx_gtk.c`, Lines 946-951

Three field definitions added:
```c
{"#apf_plugin", do_toggle_option, 1000, -1000, 40, 40, "APF", 40, "OFF", FIELD_TOGGLE, ...}
{"#apf_gain", do_apf_edit, 1000, -1000, 40, 40, "GAIN", 80, "6", FIELD_NUMBER, ...}
{"#apf_width", do_apf_edit, 1000, -1000, 40, 40, "WIDTH", 80, "100", FIELD_NUMBER, ...}
```

#### UI Positioning
**File:** `src/sbitx_gtk.c`, Function `menu_display()`, Lines 3707, 3721-3722

```c
field_move("APF", 550, screen_height - 80, 45, 37);      // Line 1
field_move("GAIN", 550, screen_height - 40, 45, 37);     // Line 2
field_move("WIDTH", 600, screen_height - 40, 45, 37);    // Line 2
```

#### Handler Function
**File:** `src/sbitx_gtk.c`, Lines 5672-5692

The `do_apf_edit()` function:
- Reads values from `#apf_gain` and `#apf_width` fields when APF is ON
- Updates `apf1.gain` and `apf1.width` in the global struct
- Sets `apf1.ison = 1` to enable the filter
- Calls `init_apf()` to recalculate filter coefficients
- Sets `apf1.ison = 0` when APF is OFF

#### Status Monitoring
**File:** `src/sbitx_gtk.c`, Lines 5892, 5987-6000

Integrated into `check_plugin_controls()` function to monitor toggle state changes in real-time.

### Backend Integration

The UI controls connect to existing APF infrastructure:

1. **Data Structure:** `struct apf apf1` (defined in sbitx_gtk.c, line 498)
   ```c
   struct apf {
       int ison;           // 0=off, 1=on
       float gain;         // dB (0-20)
       float width;        // Width parameter (10-500)
       float coeff[10];    // Filter coefficients
   };
   ```

2. **Initialization:** `init_apf()` function (sbitx_gtk.c, lines 501-519)
   - Calculates APF filter coefficients from gain and width parameters
   - Called automatically when settings change

3. **DSP Application:** Filter applied in sbitx.c (lines 1420-1441)
   - Multiplies FFT bins by precomputed coefficients when `apf1.ison==1`

## Usage

### UI Controls

1. **Enable APF:**
   - Click the APF toggle button in Menu 1
   - Toggle switches from OFF to ON
   - Filter becomes active

2. **Adjust Gain:**
   - Click the GAIN control
   - Adjust value from 0 to 20 dB
   - Default is 6 dB

3. **Adjust Width:**
   - Click the WIDTH control
   - Adjust value from 10 to 500
   - Default is 100

### Console Commands

The APF can also be controlled via console commands:

```bash
apf                    # Disable APF
apf 6 100             # Enable APF with gain=6dB, width=100
```

## Configuration

### Default Settings
**File:** `data/default_settings.ini`

```ini
#apf_plugin=OFF
#apf_gain=6
#apf_width=100
```

### User Settings

Settings automatically persist in `~/sbitx/data/user_settings.ini` when changed through the UI. The values are restored on application restart.

## Design Pattern

This implementation follows the same design pattern as the NOTCH filter:

- Toggle control on line 1 for ON/OFF
- Two numeric controls on line 2 for parameters
- Handler function to update backend struct
- Status monitoring in `check_plugin_controls()`
- Settings persistence via .ini files

## Testing

### Basic Functionality Test

1. Build and run sbitx:
   ```bash
   ./build sbitx
   ./sbitx
   ```

2. Access Menu 1:
   - Click MENU button to display Menu 1

3. Verify APF controls:
   - APF toggle should appear to the right of TNDUR
   - GAIN and WIDTH controls should appear below

4. Test APF operation:
   - Toggle APF ON
   - Adjust GAIN and WIDTH values
   - Verify audio filtering is applied
   - Toggle APF OFF to bypass filter

### Settings Persistence Test

1. Enable APF and set custom values
2. Exit sbitx
3. Restart sbitx
4. Verify APF settings are restored

### Integration Test

- Enable both NOTCH and APF simultaneously
- Verify both filters work without conflict
- Test with other DSP features (ANR, DSP, etc.)

## Troubleshooting

### APF Not Appearing in Menu

- Verify you're viewing Menu 1 (not Menu 2)
- Check that screen width is sufficient (minimum 800px recommended)

### APF Not Applying

- Verify APF toggle is ON
- Check console output for init_apf() messages
- Verify gain and width values are in valid ranges

### Settings Not Persisting

- Check file permissions on `~/sbitx/data/user_settings.ini`
- Verify settings file is writable

## Technical Notes

### Filter Coefficients

The `init_apf()` function calculates 9 coefficients based on gain and width:
- Peak coefficient at center frequency
- Symmetric coefficients for ±1 through ±4 bins
- Uses Gaussian-style weighting based on width parameter

### Performance

- Filter applied in frequency domain (FFT bins)
- Negligible CPU overhead (simple multiplication)
- No impact on audio latency

### Compatibility

- Works alongside other DSP features
- Compatible with all modes (USB, LSB, CW, etc.)
- Settings independent per user profile

## Future Enhancements

Potential improvements for future versions:

1. Visual indicator on spectrum display (similar to NOTCH)
2. Preset buttons for common gain/width combinations
3. Auto-tune feature to optimize settings
4. Frequency-specific APF profiles

## References

- See `docs/apfdoc.pdf` for APF filter theory and design
- See `src/docs/how_gui_is_organized.txt` for UI system overview
- See `src/docs/ui_notes.txt` for UI coding conventions

## Change History

- **2025-11-13:** Initial UI implementation
  - Added APF toggle, GAIN, and WIDTH controls to Menu 1
  - Integrated with existing APF backend
  - Added default configuration values
