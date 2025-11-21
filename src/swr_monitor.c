#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fftw3.h>
#include <complex.h>
#include "swr_monitor.h"
#include "sdr_ui.h"
#include "sdr.h"

// Maximum VSWR threshold (default 3.0)
float max_vswr = 3.0f;

// Flag indicating if VSWR has been tripped (0 = normal, 1 = tripped)
int vswr_tripped = 0;

// Saved drive value (for reference, not restored per requirements)
static int saved_drive_value = 0;

/**
 * Check VSWR and handle reduction/recovery
 * vswr parameter: SWR * 10 (e.g., 30 means 3.0) - project convention
 */
void check_and_handle_vswr(int vswr)
{
	// Convert from integer representation to float (vswr / 10.0)
	float swr = vswr / 10.0f;
//	printf(" handle  "); //
	// Check if VSWR exceeds threshold and not already tripped
	if (swr > max_vswr && vswr_tripped == 0) {
//		printf(" tripped\n");  //
		char response[100];
		char drive_str[32];
		char tnpwr_str[32];
		char sdr_cmd[64];
		
		// Set tripped flag
		vswr_tripped = 1;
		
		// Get TNPWR value from #tune_power field ID
		if (get_field_value("#tune_power", tnpwr_str) == 0) {
			int tunepower = atoi(tnpwr_str);
			
			// Set tx power to TNPWR via sdr_request
			snprintf(sdr_cmd, sizeof(sdr_cmd), "tx_power=%d", tunepower);
			sdr_request(sdr_cmd, response);
			
			// Update DRIVE field in GUI to reflect reduced power
			char drive_buff[32];
			snprintf(drive_buff, sizeof(drive_buff), "%d", tunepower);
			field_set("DRIVE", drive_buff);
						
			// Update UI: set alert flag
			set_field("#vswr_alert", "1");
			
			// Set spectrum left message to "HIGH SWR" in red
			set_field("#spectrum_left_msg", "HIGH SWR");
			set_field("#spectrum_left_color", "red");
			
			// Write warning to console
			char warning_msg[128];
			snprintf(warning_msg, sizeof(warning_msg), 
			         "*VSWR WARNING: SWR %.1f exceeds threshold %.1f, reducing drive to %d\n",
			         swr, max_vswr, tunepower);
			write_console(STYLE_LOG, warning_msg);
		}
	}
	// Check if VSWR has fallen below threshold and was previously tripped
	else if (swr <= max_vswr && vswr_tripped == 1) {
		// Clear tripped flag
		vswr_tripped = 0;
		
		// Clear UI alerts
		set_field("#vswr_alert", "0");
		set_field("#spectrum_left_msg", "");
		set_field("#spectrum_left_color", "");
		
		// Write info to console
		char info_msg[128];
		snprintf(info_msg, sizeof(info_msg), 
		         "*VSWR: SWR %.1f back below threshold %.1f, UI cleared (drive NOT restored)\n",
		         swr, max_vswr);
		write_console(STYLE_LOG, info_msg);
		
		// Do NOT restore the drive value - leave it reduced for safety
	}
}

/**
 * Reset VSWR tripped state and clear UI without restoring drive
 */
void reset_vswr_tripped(void)
{
	// Clear flag
	vswr_tripped = 0;
	
	// Clear UI
	set_field("#vswr_alert", "0");
	set_field("#spectrum_left_msg", "");
	set_field("#spectrum_left_color", "");
	
	write_console(STYLE_LOG, "*VSWR: Monitor reset\n");
}

/**
 * init_vswr_monitor - Initialize VSWR monitor at startup
 *
 * Ensures all UI fields are cleared and monitor is in a known state.
 * Should be called during application initialization.
 */
void init_vswr_monitor(void)
{
	// Ensure tripped flag is off
	vswr_tripped = 0;
	
	// Clear UI fields to ensure clean startup
	set_field("#vswr_alert", "0");
	set_field("#spectrum_left_msg", "");
	set_field("#spectrum_left_color", "");
}
/*
 * reset_vswr_tripped - Reset VSWR trip status and clear UI
 *
 * Clears the vswr_tripped flag and UI alerts without restoring drive.
*/
