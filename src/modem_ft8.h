#define FT8_MAX_BUFF (12000 * 18)

void ft8_rx(int32_t *samples, int count);
void ft8_init();
void ft8_abort(bool terminate_qso);
void ft8_tx(char *message, int freq);
void ft8_tx_3f(const char* call_to, const char* call_de, const char* extra);
void ft8_poll(int tx_is_on);
float ft8_next_sample();
void ft8_call(int sel_time);
void ftx_call_or_continue(const char* line, int line_len, const text_span_semantic* spans);
