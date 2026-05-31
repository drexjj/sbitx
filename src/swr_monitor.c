#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fftw3.h>
#include <complex.h>
#include "swr_monitor.h"
#include "sdr_ui.h"
#include "sdr.h"
#include <string.h>
#include <unistd.h>
#include <gtk/gtk.h>


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
extern int           set_field(const char *id, const char *value);
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
 int call_count = 0;
void check_and_handle_vswr(int vswr)
{
	// Convert from integer representation to float (vswr / 10.0)
	float swr = vswr / 10.0f;
	// Check if VSWR exceeds threshold and not already tripped
	call_count++;
	if (swr > max_vswr && vswr_tripped == 0 && vswr_on==1) { // 

		char response[100];
		char drive_str[32];
//		char tnpwr_str[32];
		char sdr_cmd[64];
		
		// Set tripped flag
		vswr_tripped = 1;
//		printf(" tripped %d\n",vswr_tripped);  //				
			set_field("tx_power", "1");			
			// Write warning to console
			char warning_msg[128];
			snprintf(warning_msg, sizeof(warning_msg), 
			         "\n *VSWR WARNING: SWR %.1f exceeds threshold %.1f\n",
			         swr, max_vswr, 1);
			write_console(STYLE_LOG, warning_msg);
			printf("on %.1f %d\n", swr, call_count);
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
		printf("off %.1f  %d\n", swr, call_count);
		// Do NOT restore the drive value - leave it reduced for safety
	}
}


void init_vswr_monitor(void)
{
	// Ensure tripped flag is off, feature activated
	vswr_tripped = 0;
	vswr_on=1;
}

/* SWR sweep
  Power and VSWR update every 100 ms, so each step must be >= 200 ms.
   Use 250 ms to leave a little margin. */
const int settle_ms = 250;
extern int vswr;
extern int in_tx;

extern void set_rx1(int frequency);
extern void do_control_action(char *cmd);
extern int field_int(char *label);
extern int field_set(const char *label, const char *new_value);

/* band limits are loaded from hw_settings.ini into band_power[] in sbitx.c */
struct power_settings
{
	int f_start;
	int f_stop;
	int max_watts;
	double scale;
};

extern struct power_settings band_power[];

#define SWR_SWEEP_BAND_COUNT 9
#define SWR_SWEEP_MIN_SETTLE_MS 200
#define SWR_SWEEP_DEFAULT_SETTLE_MS 250
#define SWR_SWEEP_MIN_DRIVE 10

static const char *swr_sweep_band_names[SWR_SWEEP_BAND_COUNT] = {
	"80M", "60M", "40M", "30M", "20M", "17M", "15M", "12M", "10M"
};

static int g_swr_sweep_running = 0;
static int g_swr_sweep_cancel = 0;

static int find_sweep_band_index(int freq)
{
	int i;

	for (i = 0; i < SWR_SWEEP_BAND_COUNT; i++) {
		if (freq >= band_power[i].f_start && freq <= band_power[i].f_stop)
			return i;
	}

	return -1;
}

int swr_sweep_is_running(void)
{
	return g_swr_sweep_running;
}

void swr_sweep_cancel(void)
{
	if (g_swr_sweep_running)
		g_swr_sweep_cancel = 1;
}

static void swr_sweep_pump_events(void)
{
	while (gtk_events_pending())
		gtk_main_iteration_do(FALSE);
}

static int sleep_with_cancel_and_events(int total_ms)
{
	int elapsed = 0;

	while (elapsed < total_ms) {
		swr_sweep_pump_events();

		if (g_swr_sweep_cancel)
			return 1;

		usleep(50000);
		elapsed += 50;
	}

	return 0;
}


void swr_sweep(int steps)
{
	char msg[256];
	int saved_freq;
	int saved_drive;
	int saved_vswr_on;
	int was_in_tx;
	int settle_ms = SWR_SWEEP_DEFAULT_SETTLE_MS;
	int band_idx;
	int start;
	int stop;
	int i;
	int freq;

	if (steps < 2) {
		write_console(STYLE_LOG, "Usage: \\swrsweep <steps>, steps must be >= 2\n");
		return;
	}

	if (g_swr_sweep_running) {
		write_console(STYLE_LOG, "SWR sweep already running\n");
		return;
	}

	g_swr_sweep_running = 1;
	g_swr_sweep_cancel = 0;

	if (settle_ms < SWR_SWEEP_MIN_SETTLE_MS)
		settle_ms = SWR_SWEEP_MIN_SETTLE_MS;

	saved_freq = get_freq();
	saved_drive = field_int("DRIVE");
	saved_vswr_on = vswr_on;
	was_in_tx = in_tx;

	band_idx = find_sweep_band_index(saved_freq);
	if (band_idx < 0) {
		write_console(STYLE_LOG, "SWR sweep: current frequency is outside configured band limits\n");
		goto cleanup;
	}

	start = band_power[band_idx].f_start;
	stop = band_power[band_idx].f_stop;

	snprintf(msg, sizeof(msg),
		"\nSWR sweep on %s: %d to %d Hz in %d steps, %d ms per step. Press ESC to cancel.\n",
		swr_sweep_band_names[band_idx], start, stop, steps, settle_ms);
	write_console(STYLE_LOG, msg);

	/* prevent protection from dropping power mid-sweep */
	vswr_on = 0;

	/* ensure enough drive for meaningful SWR readings */
	if (saved_drive < SWR_SWEEP_MIN_DRIVE) {
		char drive_buf[16];
		snprintf(drive_buf, sizeof(drive_buf), "%d", SWR_SWEEP_MIN_DRIVE);
		field_set("DRIVE", drive_buf);
		swr_sweep_pump_events();
	}

	if (!was_in_tx) {
		do_control_action("TUNE ON");
		if (sleep_with_cancel_and_events(300))
			goto canceled;
	}

	for (i = 0; i < steps; i++) {
		if (g_swr_sweep_cancel)
			goto canceled;

		freq = start + ((long long)(stop - start) * i) / (steps - 1);
		set_rx1(freq);
		swr_sweep_pump_events();

		if (sleep_with_cancel_and_events(settle_ms))
			goto canceled;

		snprintf(msg, sizeof(msg), "%.3f MHz  SWR %.1f\n", freq/1000000., vswr / 10.0f);
		write_console(STYLE_LOG, msg);
		swr_sweep_pump_events();
	}

	write_console(STYLE_LOG, "SWR sweep complete\n");
	goto cleanup_after_tx;

canceled:
	write_console(STYLE_LOG, "SWR sweep canceled\n");

cleanup_after_tx:
	if (!was_in_tx) {
		do_control_action("TUNE OFF");
		swr_sweep_pump_events();
		usleep(100000);
	}

	set_rx1(saved_freq);
	swr_sweep_pump_events();

	{
		char drive_buf[16];
		snprintf(drive_buf, sizeof(drive_buf), "%d", saved_drive);
		field_set("DRIVE", drive_buf);
	}

	vswr_on = saved_vswr_on;

cleanup:
	g_swr_sweep_running = 0;
	g_swr_sweep_cancel = 0;
}
