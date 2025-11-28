#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fftw3.h>
#include <complex.h>
#include "swr_monitor.h"
#include "sdr_ui.h"
#include "sdr.h"
/*
define a default maxvswr of 3
initialize as enabled but not tripped

If swr is over maxvswr
  the drive is set to 1
  a message is sent to the console
  a large red "HIGH VSWR" appears on the spectrum.
  
Note - You must set drive up above 3 watts for SWR to be measured
  SWR messages not deleted until there is a SWR reading below maxvswr

You can set max_vswr from the command line
  \max_vswr value (float)
  setting it to 0 turns SWR protection off, with a console message of Caution. 
   
Internally vswr_tripped tracks whether max_vswr was exceeded 
  and vswr_on tracks whether enabled or disabled  
    
*/

// Maximum VSWR threshold (default 3.0)
float max_vswr = 3.0f;

// Flag indicating if VSWR has been tripped (0 = normal, 1 = tripped)
int vswr_tripped = 0;
// Flag indicating if feature enabled (0 = disabled, 1 = enabled)
int vswr_on=1;

/**
 * Check VSWR and handle reduction/recovery
 * vswr parameter: SWR * 10 (e.g., 30 means 3.0) - project convention
 */
void check_and_handle_vswr(int vswr)
{
	// Convert from integer representation to float (vswr / 10.0)
	float swr = vswr / 10.0f;
	// Check if VSWR exceeds threshold and not already tripped
	if (swr > max_vswr && vswr_tripped == 0 && vswr_on==1) { // 
//		printf(" tripped %d\n",vswr_on);  //
		char response[100];
		char drive_str[32];
		char tnpwr_str[32];
		char sdr_cmd[64];
		
		// Set tripped flag
		vswr_tripped = 1;
		
/*		// Get TNPWR value from #tune_power field ID
		if (get_field_value("#tune_power", tnpwr_str) == 0) {
			int tunepower = atoi(tnpwr_str);
			// Set tx power to TNPWR via sdr_request
*/			

			snprintf(sdr_cmd, sizeof(sdr_cmd), "tx_power=%d", 1);
			sdr_request(sdr_cmd, response);
			
			// Update DRIVE field in GUI to reflect reduced power
			char drive_buff[32];
			snprintf(drive_buff, sizeof(drive_buff), "%d", 1);
			field_set("DRIVE", drive_buff);
						
			// Update UI: set alert flag
			set_field("#vswr_alert", "1");
			
			// Set message to "HIGH SWR" in red
			set_field("#high_vswr_msg", "HIGH VSWR");
			set_field("#high_vswr_color", "red");
			
			// Write warning to console
			char warning_msg[128];
			snprintf(warning_msg, sizeof(warning_msg), 
			         "\n *VSWR WARNING: SWR %.1f exceeds threshold %.1f, reducing drive to %d\n",
			         swr, max_vswr, 1);
			write_console(STYLE_LOG, warning_msg);
		}
	// Check if VSWR has fallen below threshold and was previously tripped
	else if (swr <= max_vswr && vswr_tripped == 1) {
		// Clear tripped flag
		vswr_tripped = 0;
		
		// Clear UI alerts
		set_field("#vswr_alert", "0");
		set_field("#high_vswr_msg", "");
		set_field("#high_vswr_color", "");
		
		// Write info to console
		char info_msg[128];
		snprintf(info_msg, sizeof(info_msg), 
			 "\n *VSWR: SWR %.1f back below threshold %.1f, UI cleared\n",
			 swr, max_vswr);
		write_console(STYLE_LOG, info_msg);
		
		// Do NOT restore the drive value - leave it reduced for safety
	}
}


void init_vswr_monitor(void)
{
	// Ensure tripped flag is off, feature activated
	vswr_tripped = 0;
	vswr_on=1;
	
	// Clear UI fields to ensure clean startup
	set_field("#vswr_alert", "0");
	set_field("#high_vswr_msg", "");
	set_field("#high_vswr_color", "");
}
