#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fftw3.h>
#include <complex.h>
#include "swr_monitor.h"
#include "sdr_ui.h"
#include "sdr.h"
/*
define a default maxv_swr of 3
initialize as enabled but not tripped
* nore program variable max_vswr - user label maxvswr

If swr is over maxv_swr
  the drive is set to 1
  a message is sent to the console
  a large red "HIGH VSWR" appears on the spectrum.
  
Note - You must set drive up above 3 watts for SWR to be measured
  SWR messages not deleted until there is a SWR reading below maxvswr

You can set max_vswr from the command line
  \maxvswr value (float)
  setting it to 0 turns SWR protection off, with a console message 
  maxvswr is saved in user_settings.ini 
     and read at startup overwriting default
   
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

		char response[100];
		char drive_str[32];
//		char tnpwr_str[32];
		char sdr_cmd[64];
		
		// Set tripped flag
		vswr_tripped = 1;
//		printf(" tripped %d\n",vswr_tripped);  //				

			snprintf(sdr_cmd, sizeof(sdr_cmd), "tx_power=%d", 1);
			sdr_request(sdr_cmd, response);
			
			// Update DRIVE field in GUI to reflect reduced power
			char drive_buff[32];
			snprintf(drive_buff, sizeof(drive_buff), "%d", 1);
			field_set("DRIVE", drive_buff);
						
			// Write warning to console
			char warning_msg[128];
			snprintf(warning_msg, sizeof(warning_msg), 
			         "\n *VSWR WARNING: SWR %.1f exceeds threshold %.1f\n",
			         swr, max_vswr, 1);
			write_console(STYLE_LOG, warning_msg);
		}
	// Check if VSWR has fallen below threshold and was previously tripped
	else if (swr <= max_vswr && vswr_tripped == 1) {
		// Clear tripped flag
		vswr_tripped = 0;
		
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
}
