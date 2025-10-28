void cw_rx(int *samples, int count);
float cw_tx_get_sample();
void cw_init();
void cw_abort();
void cw_tx(char *message, int freq);
void cw_poll(int bytes_available, int tx_is_on);
float cw_next_sample();

// added to support zerobeat display of cw decoder status
int cw_get_max_bin_highlight_index(void);

#define N_BINS 128
#define INIT_TONE 600
#define SAMPLING_FREQ 12000
#define INIT_WPM 20 
