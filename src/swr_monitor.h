#ifndef SWR_MONITOR_H
#define SWR_MONITOR_H

// Maximum VSWR threshold (default 3.0)
extern float max_vswr;

// Flag indicating if VSWR has been tripped
extern int vswr_tripped;
// Flag indicating if SWR protection enabled
extern int vswr_on;

// Initialize VSWR monitor at startup
void init_vswr_monitor(void);

// Check VSWR and handle reduction/recovery
// vswr parameter: SWR * 10 (e.g., 30 means 3.0) - project convention
void check_and_handle_vswr(int vswr);

// Reset VSWR tripped state and clear UI
void reset_vswr_tripped(void);

#endif // SWR_MONITOR_H
