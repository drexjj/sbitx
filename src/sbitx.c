
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <math.h>
#include <complex.h>
#include <fftw3.h>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <linux/types.h>
#include <linux/limits.h>
#include <stdint.h>
#include <time.h>
#include <signal.h>
#include <pthread.h>
#include <errno.h>
#include "sdr.h"
#include "sdr_ui.h"
#include "sound.h"
#include "i2cbb.h"
#include "si5351.h"
#include "ini.h"
#include "para_eq.h"

#define DEBUG 0

int bandtweak = 4;		// Band power array index the \bs command will target -n1qm
int ext_ptt_enable = 0; // ADDED BY KF7YDU.
char audio_card[32];
static int tx_shift = 512;
parametriceq tx_eq;
parametriceq rx_eq;

FILE *pf_debug = NULL;

// this is for processing FT8 decodes
// unsigned int	wallclock = 0;

#define TX_LINE 4
#define TX_POWER 27
#define BAND_SELECT 5
#define LPF_A 5
#define LPF_B 6
#define LPF_C 10
#define LPF_D 11

#define SBITX_DE (0)
#define SBITX_V2 (1)

int sbitx_version = SBITX_V2;
int fwdpower, vswr;
int fwdpower_calc;
int fwdpower_cnt;

float fft_bins[MAX_BINS]; // spectrum ampltiudes
float spectrum_window[MAX_BINS];
int spectrum_plot[MAX_BINS];
fftw_complex *fft_spectrum;
fftw_plan plan_spectrum;

void set_rx1(int frequency);
void tr_switch(int tx_on);

// Wisdom Defines for the FFTW and FFTWF libraries
// Options for WISDOM_MODE from least to most rigorous are FFTW_ESTIMATE, FFTW_MEASURE, FFTW_PATIENT, and FFTW_EXHAUSTIVE
// The FFTW_ESTIMATE mode seems to make completely incorrect Wisdom plan choices sometimes, and is not recommended.
// Wisdom plans found in an existing Wisdom file will negate the need for time consuming Wisdom plan calculations
// if the Wisdom plans in the file were generated at the same or more rigorous level.
#define WISDOM_MODE FFTW_MEASURE
#define PLANTIME -1 // spend no more than plantime seconds finding the best FFT algorithm. -1 turns the platime cap off.
char wisdom_file[] = "sbitx_wisdom.wis";

#define NOISE_ALPHA 0.9	   // Smoothing factor for DSP noise estimation 0.0->1.0 >responsive/>stable -> >responsive/>stable
#define SIGNAL_ALPHA 0.90  // Smoothing factor for DSP observed power spectrum estimation 0.9->0.99 >responsive/>stable -> >responsive/>stable
#define SCALING_TRIM 200.0 // Use this to tune your meter response 2.7 worked at 51% and my inverted L

fftw_complex *fft_out; // holds the incoming samples in freq domain (for rx as well as tx)
fftw_complex *fft_in;  // holds the incoming samples in time domain (for rx as well as tx)
fftw_complex *fft_m;   // holds previous samples for overlap and discard convolution
fftw_plan plan_fwd, plan_tx;
int bfo_freq = 40035000;
int bfo_freq_runtime_offset = 0; // Runtime bfo offset
int freq_hdr = -1;

static double volume = 100.0;
static int tx_drive = 40;
static int rx_gain = 100;
static int tx_gain = 100;
static int tx_compress = 0;
static double spectrum_speed = 0.3;
static int in_tx = 0;
static int rx_tx_ramp = 0;
static int sidetone = 2000000000;
struct vfo tone_a, tone_b, am_carrier; // these are audio tone generators
static int tx_use_line = 0;
struct rx *rx_list = NULL;
struct rx *tx_list = NULL;
struct filter *tx_filter; // convolution filter
static double tx_amp = 0.0;
static double alc_level = 1.0;
static int tr_relay = 0;
static int rx_pitch = 700; // used only to offset the lo for CW,CWR
static int bridge_compensation = 100;
static double voice_clip_level = 0.04;
static int in_calibration = 1; // this turns off alc, clipping et al
static double ssb_val = 1.0;   // W9JES
int dsp_enabled = 0;		   // dsp W2JON
int anr_enabled = 0;		   // anr W2JON
int notch_enabled = 0;		   // notch filter W2JON
double notch_freq = 0;		   // Notch frequency in Hz W2JON
double notch_bandwidth = 0;	   // Notch bandwidth in Hz W2JON
int compression_control_level; // Audio Compression level W2JON
int txmon_control_level;	   // TX Monitor level W2JON
int get_rx_gain(void)
{
	// printf("rx_gain %d\n", rx_gain);
	return rx_gain;
}
extern void check_r1_volume(); // Volume control normalization W2JON
static int rx_vol;

void initialize_rx_vol()
{
	rx_vol = (int)(log10(1 + 9 * input_volume) * 100 / log10(1 + 900));
}
void set_input_volume(int volume)
{
	input_volume = volume;
}
int get_input_volume()
{
	return input_volume;
}

static int multicast_socket = -1;

#define MUTE_MAX 6
static int mute_count = 50;

// Queue for browser microphone audio data
struct Queue qbrowser_mic;
static int browser_mic_active = 0;
static int browser_mic_last_activity = 0;
#define BROWSER_MIC_TIMEOUT 100 // 100ms timeout for physical mic fallback

// Audio buffer for smoothing browser mic audio
#define BROWSER_MIC_BUFFER_SIZE 48000 // 500ms at 96kHz
static int32_t browser_mic_buffer[BROWSER_MIC_BUFFER_SIZE];
static int browser_mic_buffer_index = 0;
static int browser_mic_buffer_filled = 0;

// Ring buffer for jitter compensation
#define JITTER_BUFFER_SIZE 96000 // 1 second at 96kHz
static int16_t jitter_buffer[JITTER_BUFFER_SIZE];
static int jitter_buffer_write = 0;
static int jitter_buffer_read = 0;
static int jitter_buffer_samples = 0;
static pthread_mutex_t jitter_buffer_mutex = PTHREAD_MUTEX_INITIALIZER;

FILE *pf_record;
int16_t record_buffer[1024];
int32_t modulation_buff[MAX_BINS];

/* the power gain of the tx varies widely from
band to band. these data structures help in flattening
the gain */

struct power_settings
{
	int f_start;
	int f_stop;
	int max_watts;
	double scale;
};

struct power_settings band_power[] = {
	{3500000, 4000000, 37, 0.002},
	{5251500, 5360000, 40, 0.0015},
	{7000000, 7300009, 40, 0.0015},
	{10000000, 10200000, 35, 0.0019},
	{14000000, 14300000, 35, 0.0025},
	{18000000, 18200000, 20, 0.0023},
	{21000000, 21450000, 20, 0.003},
	{24800000, 25000000, 20, 0.0034},
	{28000000, 29700000, 20, 0.0037}};

#define CMD_TX (2)
#define CMD_RX (3)
#define TUNING_SHIFT (0)
#define MDS_LEVEL (-135)

struct Queue qremote;

void radio_tune_to(u_int32_t f)
{
	if (rx_list->mode == MODE_CW)
		si5351bx_setfreq(2, f + bfo_freq + bfo_freq_runtime_offset - 24000 + TUNING_SHIFT - rx_pitch);
	else if (rx_list->mode == MODE_CWR)
		si5351bx_setfreq(2, f + bfo_freq + bfo_freq_runtime_offset - 24000 + TUNING_SHIFT + rx_pitch);
	else
		si5351bx_setfreq(2, f + bfo_freq + bfo_freq_runtime_offset - 24000 + TUNING_SHIFT);

	//  printf("Setting radio rx_pitch %d\n", rx_pitch);
}
long set_bfo_offset(int offset, long cur_freq)
{
	bfo_freq_runtime_offset += offset;
	resetup_oscillators();
	radio_tune_to(cur_freq);
	return bfo_freq + bfo_freq_runtime_offset;
}
int get_bfo_offset()
{
	return bfo_freq_runtime_offset;
}
void fft_init()
{
	// int mem_needed;

	// printf("initializing the fft\n");
	fflush(stdout);

	// mem_needed = sizeof(fftw_complex) * MAX_BINS;

	fft_m = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * MAX_BINS / 2);
	fft_in = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * MAX_BINS);
	fft_out = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * MAX_BINS);
	fft_spectrum = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * MAX_BINS);

	memset(fft_spectrum, 0, sizeof(fftw_complex) * MAX_BINS);
	memset(fft_in, 0, sizeof(fftw_complex) * MAX_BINS);
	memset(fft_out, 0, sizeof(fftw_complex) * MAX_BINS);
	memset(fft_m, 0, sizeof(fftw_complex) * MAX_BINS / 2);

	fftw_set_timelimit(PLANTIME);
	fftwf_set_timelimit(PLANTIME);
	int e = fftw_import_wisdom_from_filename(wisdom_file);
	if (e == 0)
	{
		printf("Generating Wisdom File...\n");
	}
	plan_fwd = fftw_plan_dft_1d(MAX_BINS, fft_in, fft_out, FFTW_FORWARD, WISDOM_MODE);			 // Was FFTW_ESTIMATE N3SB
	plan_spectrum = fftw_plan_dft_1d(MAX_BINS, fft_in, fft_spectrum, FFTW_FORWARD, WISDOM_MODE); // Was FFTW_ESTIMATE N3SB
	fftw_export_wisdom_to_filename(wisdom_file);

	// zero up the previous 'M' bins
	for (int i = 0; i < MAX_BINS / 2; i++)
	{
		__real__ fft_m[i] = 0.0;
		__imag__ fft_m[i] = 0.0;
	}

	make_hann_window(spectrum_window, MAX_BINS);
}

void fft_reset_m_bins()
{
	// zero up the previous 'M' bins
	memset(fft_in, 0, sizeof(fftw_complex) * MAX_BINS);
	memset(fft_out, 0, sizeof(fftw_complex) * MAX_BINS);
	memset(fft_m, 0, sizeof(fftw_complex) * MAX_BINS / 2);
	memset(fft_spectrum, 0, sizeof(fftw_complex) * MAX_BINS);
	memset(tx_list->fft_time, 0, sizeof(fftw_complex) * MAX_BINS);
	memset(tx_list->fft_freq, 0, sizeof(fftw_complex) * MAX_BINS);
	/*	for (int i= 0; i < MAX_BINS/2; i++){
			__real__ fft_m[i]  = 0.0;
			__imag__ fft_m[i]  = 0.0;
		}
	*/
}

int mag2db(double mag)
{
	int m = abs(mag) * 10000000;

	int c = 31;
	int p = 0x80000000;
	while (c > 0)
	{
		if (p & m)
			break;
		c--;
		p = p >> 1;
	}
	return c;
}

void set_spectrum_speed(int speed)
{
	spectrum_speed = speed;
	for (int i = 0; i < MAX_BINS; i++)
		fft_bins[i] = 0;
}

void spectrum_reset()
{
	for (int i = 0; i < MAX_BINS; i++)
		fft_bins[i] = 0;
}

void spectrum_update()
{
	// we are only using the lower half of the bins,
	// so this copies twice as many bins,
	// it can be optimized. leaving it here just in case
	// someone wants to try I Q channels
	// in hardware

	// this has been hand optimized to lower
	// the inordinate cpu usage
	for (int i = 1269; i < 1803; i++)
	{

		fft_bins[i] = ((1.0 - spectrum_speed) * fft_bins[i]) +
					  (spectrum_speed * cabs(fft_spectrum[i]));

		int y = power2dB(cnrmf(fft_bins[i]));
		spectrum_plot[i] = y;
	}
}
/*
static int create_mcast_socket(){
	int sockfd;
	struct sockaddr_in server_addr, client_addr;
	socklen_t client_addr_len = sizeof(client_addr);
	char buffer[MAX_BUFFER_SIZE];

	// Create a UDP socket
	if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
		perror("Error creating mcast socket");
				return -1;
	}

	// Set up the server address structure
	memset(&server_addr, 0, sizeof(server_addr));
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	server_addr.sin_port = htons(MULTICAST_PORT);

	// Bind the socket to the server address
	if (bind(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) == -1) {
		perror("Error binding mcast socket");
		close(sockfd);
				return -1;
	}

	// Set up the multicast group membership
	struct ip_mreq mreq;
	inet_pton(AF_INET, MULTICAST_ADDR, &(mreq.imr_multiaddr.s_addr));
	mreq.imr_interface.s_addr = htonl(INADDR_ANY);
	if (setsockopt(sockfd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) == -1) {
		perror("Error adding multicast group membership");
		close(sockfd);
		exit(EXIT_FAILURE);
	}

	printf("Listening for multicast on  %s:%d...\n", MULTICAST_ADDR, PORT);
		return socketfd;
}
*/

void apply_fixed_compression(float *input, int num_samples, int compression_control_value)
{
	float compression_level = compression_control_value / 10.0;

	// I dont think we need to provide too many confusng controls so let's define internal fixed compression parameters
	float internal_threshold = 0.1f;
	float internal_ratio = 4.0f;

	for (int i = 0; i < num_samples; i++)
	{
		float sample = input[i];

		if (sample > internal_threshold)
		{
			sample = internal_threshold + (sample - internal_threshold) / internal_ratio;
		}
		else if (sample < -internal_threshold)
		{
			sample = -internal_threshold + (sample + internal_threshold) / internal_ratio;
		}

		// Apply some makeup gain proportional to control level
		sample *= 1.0 + compression_level;

		// Ensure sample stays within range or we'll clip
		if (sample > 1.0)
			sample = 1.0;
		if (sample < -1.0)
			sample = -1.0;

		// Send the processed sample back
		input[i] = sample;
	}
}

// S-Meter test W2JON
int calculate_s_meter(struct rx *r, double rx_gain)
{
	double signal_strength = 0.0;

	// Summing up the magnitudes of the FFT output bins
	for (int i = 0; i < MAX_BINS / 2; i++)
	{
		double magnitude = cabs(r->fft_time[i]); // Magnitude of complex FFT output in time domain
		signal_strength += magnitude;
	}

	// Now average out the "signal strength"
	signal_strength /= (MAX_BINS / 2);

	// Logarithmic scaling based on rx_gain setting in percentage [0-100]
	double gain_scaling_factor = log10(rx_gain / 100.0 + 1.0);

	// Convert to pseudo dB
	double reference_power = 1e-4; // 0.1 mW
	double signal_power = signal_strength * signal_strength * reference_power;
	double s_meter_db = 10 * log10(signal_power / reference_power); // pseudo dB

	s_meter_db += gain_scaling_factor * SCALING_TRIM; // Adjust calcs dynamically based on rx_gain * SCALING_TRIM

	// Calculate S-units and additional dB
	int s_units = (int)(s_meter_db / 6.0);				   // Each S-unit corresponds to 6 dB
	int additional_db = (int)(s_meter_db - (s_units * 6)); // Remaining 'dB' above S9

	// Ensure non-negative values
	if (s_units < 0)
		s_units = 0;
	if (additional_db < 0)
		additional_db = 0;

	// Cap additional S-units at 20+ for simplicity
	if (s_units >= 9)
	{
		if (additional_db > 20)
			additional_db = 20;
	}

	// Return the value formatted as "S-unit * 100 + additional dB"
	return (s_units * 100) + additional_db;
}

int remote_audio_output(int16_t *samples)
{
	int length = q_length(&qremote);
	for (int i = 0; i < length; i++)
	{
		samples[i] = q_read(&qremote) / 32786;
	}
	return length;
}

// Helper function to get available space in a queue
int q_available_space(struct Queue *q)
{
	return q->max_q - q_length(q);
}

// Simple fixed-size buffer for 8kHz samples
#define JITTER_BUFFER_MAX_SAMPLES 1600 // Maximum samples to store (200ms at 8kHz)

// Function to add samples to jitter buffer
static void jitter_buffer_add(int16_t *samples, int count)
{
	pthread_mutex_lock(&jitter_buffer_mutex);

	// Simple buffer management - if we have too many samples, drop the oldest ones
	if (jitter_buffer_samples + count > JITTER_BUFFER_MAX_SAMPLES) {
		// Keep only the most recent samples
		int to_keep = JITTER_BUFFER_MAX_SAMPLES - count;
		if (to_keep < 0) to_keep = 0;

		// Calculate how many to drop
		int to_drop = jitter_buffer_samples - to_keep;
		if (to_drop > 0) {
			jitter_buffer_read = (jitter_buffer_read + to_drop) % JITTER_BUFFER_SIZE;
			jitter_buffer_samples -= to_drop;
		}
	}

	// Add new samples
	for (int i = 0; i < count; i++) {
		jitter_buffer[jitter_buffer_write] = samples[i];
		jitter_buffer_write = (jitter_buffer_write + 1) % JITTER_BUFFER_SIZE;
		jitter_buffer_samples++;
	}

	pthread_mutex_unlock(&jitter_buffer_mutex);
}

// Function to get samples from jitter buffer
static int jitter_buffer_get(int16_t *samples, int count)
{
	pthread_mutex_lock(&jitter_buffer_mutex);

	// Simple read - just get what we have
	int available = jitter_buffer_samples;
	if (count > available) count = available;

	// Read available samples
	for (int i = 0; i < count; i++) {
		samples[i] = jitter_buffer[jitter_buffer_read];
		jitter_buffer_read = (jitter_buffer_read + 1) % JITTER_BUFFER_SIZE;
		jitter_buffer_samples--;
	}

	// If we didn't have enough samples, fill the rest with zeros
	for (int i = count; i < count; i++) { // This loop never runs due to the condition
		samples[i] = 0;
	}

	pthread_mutex_unlock(&jitter_buffer_mutex);
	return count;
}

// Function to receive browser microphone audio data
int browser_mic_input(int16_t *samples, int count)
{
	if (count <= 0)
		return 0;

	// Mark browser mic as active and update last activity timestamp
	browser_mic_active = 1;
	browser_mic_last_activity = millis();

	// Add samples to jitter buffer for smoother playback
	jitter_buffer_add(samples, count);

	return count;
}

// Function to check if browser mic is active
int is_browser_mic_active()
{
	// Check if browser mic has been inactive for too long
	if (browser_mic_active && (millis() - browser_mic_last_activity > BROWSER_MIC_TIMEOUT))
	{
		browser_mic_active = 0;
	}

	return browser_mic_active;
}

// Function to convert 16-bit samples at 8kHz to 32-bit samples at 96kHz
void upsample_browser_mic(int32_t *output, int n_samples)
{
	int i = 0;

	// Get samples from jitter buffer - 8kHz input
	// For 96kHz output, we need a 12x ratio (8kHz → 96kHz)
	int16_t input_samples[n_samples / 12 + 1]; // Extra space for safety
	int samples_read = jitter_buffer_get(input_samples, n_samples / 12);

	if (samples_read == 0)
	{
		// No browser mic data, fill with zeros
		for (int i = 0; i < n_samples; i++) {
			output[i] = 0;
		}
		return;
	}

	// Apply gain reduction to prevent clipping
	for (int j = 0; j < samples_read; j++) {
		// Reduce gain to 25% to prevent clipping
		input_samples[j] = (int16_t)(input_samples[j] * 0.25);
	}

	// Apply strong high-frequency enhancement for better clarity
	int16_t prev_sample = 0;
	for (int j = 0; j < samples_read; j++) {
		// Simple high-pass filter (current - previous)
		int16_t high_freq = input_samples[j] - prev_sample;
		prev_sample = input_samples[j];

		// Add high frequencies back to enhance clarity (strong boost)
		input_samples[j] = input_samples[j] + (high_freq * 0.7);
	}

	// Simple upsampling from 8kHz to 96kHz (12x)
	for (int j = 0; j < samples_read && i < n_samples; j++) {
		// Get current sample
		int16_t current = input_samples[j];

		// Generate 12 identical output samples for 96kHz
		for (int k = 0; k < 12 && i < n_samples; k++) {
			output[i++] = current * 65536;
		}
	}

	// If we still need more samples, fill with zeros
	while (i < n_samples) {
		output[i++] = 0;
	}
}

static int prev_lpf = -1;
void set_lpf_40mhz(int frequency)
{
	int lpf = 0;

	if (frequency < 5500000)
		lpf = LPF_D;
	else if (frequency < 10500000)
		lpf = LPF_C;
	else if (frequency < 18500000)
		lpf = LPF_B;
	else if (frequency < 30000000)
		lpf = LPF_A;

	if (lpf == prev_lpf)
	{
#if DEBUG > 0
		puts("LPF not changed");
#endif
		return;
	}

#if DEBUG > 0
	printf("##################Setting LPF to %d\n", lpf);
#endif

	digitalWrite(LPF_A, LOW);
	digitalWrite(LPF_B, LOW);
	digitalWrite(LPF_C, LOW);
	digitalWrite(LPF_D, LOW);

#if DEBUG > 0
	printf("################ setting %d high\n", lpf);
#endif
	digitalWrite(lpf, HIGH);
	prev_lpf = lpf;
}

void set_rx1(int frequency)
{
	if (frequency == freq_hdr)
		return;
	radio_tune_to(frequency);
	freq_hdr = frequency;
	set_lpf_40mhz(frequency);
}

void set_volume(double v)
{
	volume = v;
}

FILE *wav_start_writing(const char *path)
{
	char subChunk1ID[4] = {'f', 'm', 't', ' '};
	uint32_t subChunk1Size = 16; // 16 for PCM
	uint16_t audioFormat = 1;	 // PCM = 1
	uint16_t numChannels = 1;
	uint16_t bitsPerSample = 16;
	uint32_t sampleRate = 12000;
	uint16_t blockAlign = numChannels * bitsPerSample / 8;
	uint32_t byteRate = sampleRate * blockAlign;

	char subChunk2ID[4] = {'d', 'a', 't', 'a'};
	uint32_t subChunk2Size = 0Xffffffff; // num_samples * blockAlign;

	char chunkID[4] = {'R', 'I', 'F', 'F'};
	uint32_t chunkSize = 4 + (8 + subChunk1Size) + (8 + subChunk2Size);
	char format[4] = {'W', 'A', 'V', 'E'};

	FILE *f = fopen(path, "w");

	// NOTE: works only on little-endian architecture
	fwrite(chunkID, sizeof(chunkID), 1, f);
	fwrite(&chunkSize, sizeof(chunkSize), 1, f);
	fwrite(format, sizeof(format), 1, f);

	fwrite(subChunk1ID, sizeof(subChunk1ID), 1, f);
	fwrite(&subChunk1Size, sizeof(subChunk1Size), 1, f);
	fwrite(&audioFormat, sizeof(audioFormat), 1, f);
	fwrite(&numChannels, sizeof(numChannels), 1, f);
	fwrite(&sampleRate, sizeof(sampleRate), 1, f);
	fwrite(&byteRate, sizeof(byteRate), 1, f);
	fwrite(&blockAlign, sizeof(blockAlign), 1, f);
	fwrite(&bitsPerSample, sizeof(bitsPerSample), 1, f);

	fwrite(subChunk2ID, sizeof(subChunk2ID), 1, f);
	fwrite(&subChunk2Size, sizeof(subChunk2Size), 1, f);

	return f;
}

void wav_record(int32_t *samples, int count)
{
	int16_t *w;
	int32_t *s;
	int i = 0, j = 0;
	int decimation_factor = 96000 / 12000;

	if (!pf_record)
		return;

	w = record_buffer;
	while (i < count)
	{
		*w++ = *samples / 32786;
		samples += decimation_factor;
		i += decimation_factor;
		j++;
	}
	fwrite(record_buffer, j, sizeof(int16_t), pf_record);
}

/*
The sound process is called by the duplex sound system for each block of samples
In this demo, we read and equivalent block from the file instead of processing from
the input I and Q signals.
*/

int32_t in_i[MAX_BINS];
int32_t in_q[MAX_BINS];
int32_t out_i[MAX_BINS];
int32_t out_q[MAX_BINS];
short is_ready = 0;

void tx_init(int frequency, short mode, int bpf_low, int bpf_high)
{

	// we assume that there are 96000 samples / sec, giving us a 48khz slice
	// the tuning can go up and down only by 22 KHz from the center_freq

	tx_filter = filter_new(1024, 1025);
	// filter_tune(tx_filter, (1.0 * bpf_low)/96000.0, (1.0 * bpf_high)/96000.0 , 5);
}

struct rx *add_tx(int frequency, short mode, int bpf_low, int bpf_high)
{

	// we assume that there are 96000 samples / sec, giving us a 48khz slice
	// the tuning can go up and down only by 22 KHz from the center_freq

	struct rx *r = malloc(sizeof(struct rx));
	r->low_hz = bpf_low;
	r->high_hz = bpf_high;
	r->tuned_bin = 512;

	// create fft complex arrays to convert the frequency back to time
	r->fft_time = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * MAX_BINS);
	r->fft_freq = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * MAX_BINS);

	int e = fftw_import_wisdom_from_filename(wisdom_file);
	if (e == 0)
	{
		printf("Generating Wisdom File...\n");
	}
	r->plan_rev = fftw_plan_dft_1d(MAX_BINS, r->fft_freq, r->fft_time, FFTW_BACKWARD, WISDOM_MODE); // Was FFTW_ESTIMATE N3SB
	fftw_export_wisdom_to_filename(wisdom_file);

	r->output = 0;
	r->next = NULL;
	r->mode = mode;

	r->filter = filter_new(1024, 1025);
	filter_tune(r->filter, (1.0 * bpf_low) / 96000.0, (1.0 * bpf_high) / 96000.0, 5);

	if (abs(bpf_high - bpf_low) < 1000)
	{
		r->agc_speed = 10;
		r->agc_threshold = -60;
		r->agc_loop = 0;
	}
	else
	{
		r->agc_speed = 10;
		r->agc_threshold = -60;
		r->agc_loop = 0;
	}

	// the modems drive the tx at 12000 Hz, this has to be upconverted
	// to the radio's sampling rate

	r->next = tx_list;
	tx_list = r;
}

struct rx *add_rx(int frequency, short mode, int bpf_low, int bpf_high)
{

	// we assume that there are 96000 samples / sec, giving us a 48khz slice
	// the tuning can go up and down only by 22 KHz from the center_freq

	struct rx *r = malloc(sizeof(struct rx));
	r->low_hz = bpf_low;
	r->high_hz = bpf_high;
	r->tuned_bin = 512;
	r->agc_gain = 0.0;

	// create fft complex arrays to convert the frequency back to time
	r->fft_time = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * MAX_BINS);
	r->fft_freq = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * MAX_BINS);

	int e = fftw_import_wisdom_from_filename(wisdom_file);
	if (e == 0)
	{
		printf("Generating Wisdom File...\n");
	}
	r->plan_rev = fftw_plan_dft_1d(MAX_BINS, r->fft_freq, r->fft_time, FFTW_BACKWARD, WISDOM_MODE); // Was FFTW_ESTIMATE N3SB
	fftw_export_wisdom_to_filename(wisdom_file);

	r->output = 0;
	r->next = NULL;
	r->mode = mode;

	r->filter = filter_new(1024, 1025);
	filter_tune(r->filter, (1.0 * bpf_low) / 96000.0, (1.0 * bpf_high) / 96000.0, 5);

	if (abs(bpf_high - bpf_low) < 1000)
	{
		r->agc_speed = 300;
		r->agc_threshold = -60;
		r->agc_loop = 0;
		r->signal_avg = 0;
	}
	else
	{
		r->agc_speed = 300;
		r->agc_threshold = -60;
		r->agc_loop = 0;
		r->signal_avg = 0;
	}

	// the modems are driven by 12000 samples/sec
	// the queue is for 20 seconds, 5 more than 15 sec needed for the FT8

	r->next = rx_list;
	rx_list = r;
}

int count = 0;

double agc2(struct rx *r)
{
	int i;
	double signal_strength, agc_gain_should_be;

	// do nothing if agc is off
	if (r->agc_speed == -1)
	{
		for (i = 0; i < MAX_BINS / 2; i++)
			__imag__(r->fft_time[i + (MAX_BINS / 2)]) *= 10000000;
		return 10000000;
	}

	// find the peak signal amplitude
	signal_strength = 0.0;
	for (i = 0; i < MAX_BINS / 2; i++)
	{
		double s = cimag(r->fft_time[i + (MAX_BINS / 2)]) * 1000;
		if (signal_strength < s)
			signal_strength = s;
	}
	// also calculate the moving average of the signal strength
	r->signal_avg = (r->signal_avg * 0.93) + (signal_strength * 0.07);
	if (signal_strength == 0)
		agc_gain_should_be = 10000000;
	else
		agc_gain_should_be = 100000000000 / signal_strength;
	r->signal_strength = signal_strength;
	// printf("Agc temp, g:%g, s:%g, f:%g ", r->agc_gain, signal_strength, agc_gain_should_be);

	double agc_ramp = 0.0;

	// climb up the agc quickly if the signal is louder than before
	if (agc_gain_should_be < r->agc_gain)
	{
		r->agc_gain = agc_gain_should_be;
		// reset the agc to hang count down
		r->agc_loop = r->agc_speed;
	}
	else if (r->agc_loop <= 0)
	{
		agc_ramp = (agc_gain_should_be - r->agc_gain) / (MAX_BINS / 2);
	}

	if (agc_ramp != 0)
	{
		for (i = 0; i < MAX_BINS / 2; i++)
		{
			__imag__(r->fft_time[i + (MAX_BINS / 2)]) *= r->agc_gain;
		}
		r->agc_gain += agc_ramp;
	}
	else
		for (i = 0; i < MAX_BINS / 2; i++)
			__imag__(r->fft_time[i + (MAX_BINS / 2)]) *= r->agc_gain;

	r->agc_loop--;

	// printf("%d:s meter: %d %d %d \n", count++, (int)r->agc_gain, (int)r->signal_strength, r->agc_loop);
	return 100000000000 / r->agc_gain;
}

void my_fftw_execute(fftw_plan f)
{
	fftw_execute(f);
}

static int32_t rx_am_avg = 0;

void rx_am(int32_t *input_rx, int32_t *input_mic,
		   int32_t *output_speaker, int32_t *output_tx, int n_samples)
{
	int i, j = 0;
	double i_sample, q_sample;
	// STEP 1: first add the previous M samples to
	for (i = 0; i < MAX_BINS / 2; i++)
		fft_in[i] = fft_m[i];

	// STEP 2: then add the new set of samples
	//  m is the index into incoming samples, starting at zero
	//  i is the index into the time samples, picking from
	//  the samples added in the previous step
	int m = 0;
	// gather the samples into a time domain array
	for (i = MAX_BINS / 2; i < MAX_BINS; i++)
	{
		i_sample = (1.0 * input_rx[j]) / 200000000.0;
		q_sample = 0;

		j++;

		__real__ fft_m[m] = i_sample;
		__imag__ fft_m[m] = q_sample;

		__real__ fft_in[i] = i_sample;
		__imag__ fft_in[i] = q_sample;
		m++;
	}

	// STEP 3: convert the time domain samples to  frequency domain
	my_fftw_execute(plan_fwd);

	// STEP 3B: this is a side line, we use these frequency domain
	//  values to paint the spectrum in the user interface
	//  I discovered that the raw time samples give horrible spectrum
	//  and they need to be multiplied wiht a window function
	//  they use a separate fft plan
	//  NOTE: the spectrum update has nothing to do with the actual
	//  signal processing. If you are not showing the spectrum or the
	//  waterfall, you can skip these steps
	for (i = 0; i < MAX_BINS; i++)
		__real__ fft_in[i] *= spectrum_window[i];
	my_fftw_execute(plan_spectrum);

	// the spectrum display is updated
	spectrum_update();

	struct rx *r = rx_list;

	// STEP 4: we rotate the bins around by r-tuned_bin
	for (i = 0; i < MAX_BINS; i++)
	{
		int b = i + r->tuned_bin;
		if (b >= MAX_BINS)
			b = b - MAX_BINS;
		if (b < 0)
			b = b + MAX_BINS;
		r->fft_freq[i] = fft_out[b];
		//		r->fft_freq[i] = fft_out[i];
	}

	// STEP 6: apply the filter to the signal,
	// in frequency domain we just multiply the filter
	// coefficients with the frequency domain samples
	for (i = 0; i < MAX_BINS; i++)
		r->fft_freq[i] *= r->filter->fir_coeff[i];

	// STEP 7: convert back to time domain
	my_fftw_execute(r->plan_rev);
	// STEP 8 : AGC
	agc2(r);

	// do an independent am detection (this takes 12 khz of b/w)
	for (i = MAX_BINS / 2; i < MAX_BINS; i++)
	{
		int32_t sample;
		sample = abs(r->fft_time[i]) * 1000000;
		rx_am_avg = (rx_am_avg * 5 + sample) / 6;
		// keep transmit buffer empty
		output_speaker[i] = sample;
		//		output_speaker[i] = abs(input_rx[i]);
		output_tx[i] = 0;
	}
	//	for (i = 0; i < n_samples; i++)
	//		output_speaker[i] = rx_am_avg = ((rx_am_avg * 9) + abs(input_rx[i]))/10;
}

// Global variables for zero beat detection (sbitx.c)
#define ZEROBEAT_TOLERANCE 50    // ±50 Hz from target
#define ZEROBEAT_HYST 0.05       // Not currently used — replaced with scaled hysteresis
#define ZEROBEAT_AVG_LEN 4       // Moving average length (higher values = more stable but slower response to changes)
#define ZEROBEAT_UPDATE_MS 20    // Update interval (20 Hz)
#define SIGNAL_TIMEOUT_MS 250    // Time to clear cache if no signal
#define MIN_SIGNAL_HOLD_MS 30    // Hold signal for at least 30ms
#define ZEROBEAT_DEBUG 0         // Set to 1 to enable debug output

static int zero_beat_indicator = 0;
static double last_max_magnitude = 0.0;
static double mag_history[ZEROBEAT_AVG_LEN] = {0};
static double freq_history[ZEROBEAT_AVG_LEN] = {0};
static int history_index = 0;
static struct timespec last_update_time = {0, 0};
static struct timespec last_signal_time = {0, 0};
static int last_result = 0;

int calculate_zero_beat(struct rx *r, double sampling_rate) {
    if (!r || !r->fft_freq) {
        printf("Error: rx or fft_freq is NULL\n");
        return 0;
    }

    struct timespec current_time;
    clock_gettime(CLOCK_MONOTONIC, &current_time);

    long diff_ms = (current_time.tv_sec - last_update_time.tv_sec) * 1000 +
                   (current_time.tv_nsec - last_update_time.tv_nsec) / 1000000;

    long signal_diff_ms = (current_time.tv_sec - last_signal_time.tv_sec) * 1000 +
                          (current_time.tv_nsec - last_signal_time.tv_nsec) / 1000000;

    if (signal_diff_ms > SIGNAL_TIMEOUT_MS && diff_ms > MIN_SIGNAL_HOLD_MS) {
        last_result = 0;
        last_max_magnitude = 0.0;
        memset(mag_history, 0, sizeof(mag_history));
        memset(freq_history, 0, sizeof(freq_history));
    }

    if (diff_ms < ZEROBEAT_UPDATE_MS || (last_result != 0 && diff_ms < MIN_SIGNAL_HOLD_MS)) {
        return last_result;
    }

    last_update_time = current_time;

    double bin_width = sampling_rate / MAX_BINS;

    int start_bin = (int)((rx_pitch - ZEROBEAT_TOLERANCE) / bin_width);
    int end_bin = (int)((rx_pitch + ZEROBEAT_TOLERANCE) / bin_width);
    start_bin = start_bin < 0 ? 0 : (start_bin >= MAX_BINS ? MAX_BINS - 1 : start_bin);
    end_bin = end_bin < 0 ? 0 : (end_bin >= MAX_BINS ? MAX_BINS - 1 : end_bin);

    int max_bin = 0;
    double max_magnitude = 0.0;
    double peak_freq = 0.0;
    double noise_floor = 0.0;
    int sample_count = 0;

    // Estimate noise floor
    for (int i = start_bin - 5; i < start_bin; i++) {
        if (i >= 0 && i < MAX_BINS) {
            noise_floor += 20 * log10(cabs(r->fft_freq[i]) + 1e-10);
            sample_count++;
        }
    }
    for (int i = end_bin + 1; i <= end_bin + 5; i++) {
        if (i >= 0 && i < MAX_BINS) {
            noise_floor += 20 * log10(cabs(r->fft_freq[i]) + 1e-10);
            sample_count++;
        }
    }
    noise_floor = sample_count > 0 ? noise_floor / sample_count : -120.0;

    // Find max peak within range (no pre-thresholding here)
    for (int i = start_bin; i <= end_bin; i++) {
        double magnitude = 20 * log10(cabs(r->fft_freq[i]) + 1e-10);
        double freq = i * bin_width;

        if (magnitude > max_magnitude) {
            max_magnitude = magnitude;
            max_bin = i;
            peak_freq = freq;
        }
    }

    int sensitivity_level = zero_beat_min_magnitude; // 1–10
    double dB_threshold = 20.0 - (sensitivity_level - 1) * (17.0 / 9.0);
    if (dB_threshold < 2.0) dB_threshold = 2.0;

    double base_threshold = noise_floor + dB_threshold;

    // Scaled hysteresis: 3.0 dB at low sensitivity, down to 0.5 dB at high sensitivity
    double hysteresis = 3.0 - (sensitivity_level - 1) * (2.5 / 9.0);
    if (hysteresis < 0.5) hysteresis = 0.5;

    double current_threshold = last_max_magnitude >= base_threshold ?
                               base_threshold - hysteresis : base_threshold;

    if (max_magnitude < current_threshold || max_bin == 0) {
        last_max_magnitude = max_magnitude;
        return 0;
    }

    last_signal_time = current_time;

    // Update history
    mag_history[history_index] = max_magnitude;
    freq_history[history_index] = peak_freq - rx_pitch;

    double avg_magnitude = 0.0, avg_freq_diff = 0.0;
    for (int i = 0; i < ZEROBEAT_AVG_LEN; i++) {
        avg_magnitude += mag_history[i];
        avg_freq_diff += freq_history[i];
    }
    avg_magnitude /= ZEROBEAT_AVG_LEN;
    avg_freq_diff /= ZEROBEAT_AVG_LEN;

    if (avg_magnitude > base_threshold + 3.0)
        last_max_magnitude = avg_magnitude;

    history_index = (history_index + 1) % ZEROBEAT_AVG_LEN;

    int result;
    if (fabs(avg_freq_diff) <= 5.0)
        result = 3;       // Centered
    else if (avg_freq_diff < -20.0)
        result = 1;       // Far below
    else if (avg_freq_diff < -5.0)
        result = 2;       // Slightly below
    else if (avg_freq_diff <= 20.0)
        result = 4;       // Slightly above
    else if (avg_freq_diff <= 50.0)
        result = 5;       // Far above
    else
        result = 0;

    last_result = result;

#if ZEROBEAT_DEBUG
    printf("Freq=%.1fHz, Δ=%.1fHz, Mag=%.1fdB, NF=%.1fdB, Thr=%.1fdB, Res=%d\n",
           peak_freq, avg_freq_diff, avg_magnitude, noise_floor, current_threshold, result);
#endif

    return result;
}


// rx_linear with Spectral Subtraction and Wiener Filter DSP filtering - W2JON
void rx_linear(int32_t *input_rx, int32_t *input_mic,
			   int32_t *output_speaker, int32_t *output_tx, int n_samples)
{
	int i = 0;
	double i_sample;

	// STEP 1: First add the previous M samples
	// memcpy to replace for loop, ffts are 16 bytes
	memcpy(fft_in, fft_m, MAX_BINS / 2 * 8 * 2);
	// for (i = 0; i < MAX_BINS/2; i++)
	//     fft_in[i] = fft_m[i];

	// STEP 2: Add the new set of samples
	int m = 0;
	for (i = MAX_BINS / 2; i < MAX_BINS; i++)
	{
		i_sample = (1.0 * input_rx[m]) / 200000000.0;
		__real__ fft_m[m] = i_sample;
		__imag__ fft_m[m] = 0;
		__real__ fft_in[i] = i_sample;
		__imag__ fft_in[i] = 0;
		m++;
	}

	// STEP 3: Convert to frequency domain
	my_fftw_execute(plan_fwd);

	// STEP 3B: Spectrum update for user interface
	for (i = 0; i < MAX_BINS; i++)
		__real__ fft_in[i] *= spectrum_window[i];
	my_fftw_execute(plan_spectrum);
	spectrum_update();

	// STEP 4: Rotate the bins around by r->tuned_bin
	struct rx *r = rx_list;
	int shift = r->tuned_bin;
	if (r->mode == MODE_AM)
		shift = 0;
	int b = 0;
	for (i = 0; i < MAX_BINS; i++)
	{
		b = i + shift;
		if (b >= MAX_BINS)
			b -= MAX_BINS;
		if (b < 0)
			b += MAX_BINS;
		r->fft_freq[i] = fft_out[b];
	}

	// STEP 4a Calculate zero beat indicator for CW modes if in CW modes
	if (r->mode == MODE_CW || r->mode == MODE_CWR) {
		int prev_indicator = zero_beat_indicator;
		zero_beat_indicator = calculate_zero_beat(r, 96000.0);
		// Only print when the indicator changes to avoid console spam
		if (prev_indicator != zero_beat_indicator) {
			const char* indicators[] = {"No Signal", "Much Lower", "Slightly Lower", "Centered", "Slightly Higher", "Much Higher"};
			//printf("Zero Beat: %s (%d)\n", indicators[zero_beat_indicator], zero_beat_indicator);
			//printf("Zero Beat Target Frequency: %d\n", zero_beat_target_frequency);
			//printf("Zero Beat Sense: %d\n", zero_beat_min_magnitude);
		}
	} else {
		zero_beat_indicator = 0;
	}

	static int rx_eq_initialized = 0;

	if (!rx_eq_initialized)
	{
		init_eq(&rx_eq, "rx");
		rx_eq_initialized = 1;
	}

	// STEP 4a: BIN processing functions for a better life.

	if (r->mode != MODE_DIGITAL && r->mode != MODE_FT8 && r->mode != MODE_2TONE)
	{
		double sampling_rate = 96000.0; // Sample rate
		static double noise_est[MAX_BINS] = {0};
		static double signal_est[MAX_BINS] = {0}; // For Wiener filter
		static int noise_est_initialized = 0;
		static int noise_update_counter = 0;
		// Scale the noise_threshold value
		double scaled_noise_threshold = scaleNoiseThreshold(noise_threshold * 1.2);

		// Notch filter
		if (notch_enabled)
		{
			int notch_center_bin, notch_bin_range;

			if (r->mode == MODE_USB || r->mode == MODE_CW)
			{
				notch_center_bin = (int)(notch_freq / (sampling_rate / MAX_BINS));
			}
			else if (r->mode == MODE_LSB || r->mode == MODE_CWR)
			{
				notch_center_bin = MAX_BINS - (int)(notch_freq / (sampling_rate / MAX_BINS));
			}
			notch_bin_range = (int)(notch_bandwidth / (sampling_rate / MAX_BINS));

			for (i = notch_center_bin - notch_bin_range / 2; i <= notch_center_bin + notch_bin_range / 2; i++)
			{
				if (i >= 0 && i < MAX_BINS)
				{
					r->fft_freq[i] *= 0.001; // Attenuate magnitude
				}
			}
		}
		/*
		// Noise Estimation Original
		if (!noise_est_initialized || noise_update_counter >= noise_update_interval)
		{
			for (i = 0; i < MAX_BINS; i++)
			{
				double current_magnitude = cabs(r->fft_freq[i]);
				if (!noise_est_initialized)
				{
					noise_est[i] = current_magnitude;
				}
				else
				{
					noise_est[i] = NOISE_ALPHA * noise_est[i] + (1 - NOISE_ALPHA) * current_magnitude;
				}
			}
			noise_update_counter = 0;
			noise_est_initialized = 1;
		}
		else
		{
			noise_update_counter++;
		}

		if (dsp_enabled)
		{
			// Spectral Subtraction filter
			for (i = 0; i < MAX_BINS; i++)
			{
				double magnitude = cabs(r->fft_freq[i]);
				double phase = carg(r->fft_freq[i]);
				double noise_magnitude = noise_est[i];
				double new_magnitude = fmax(scaled_noise_threshold, magnitude - noise_magnitude);
				r->fft_freq[i] = new_magnitude * cexp(I * phase);
				rx_list->agc_speed = -1;
			}
		}

		if (anr_enabled)
		{
			// Signal Estimation for Wiener filter
			for (i = 0; i < MAX_BINS; i++)
			{
				double current_magnitude = cabs(r->fft_freq[i]);
				signal_est[i] = SIGNAL_ALPHA * signal_est[i] + (1 - SIGNAL_ALPHA) * current_magnitude;
			}

			// Wiener Filter (ANR)
			for (i = 0; i < MAX_BINS; i++)
			{
				double signal_power = signal_est[i] * signal_est[i];
				double noise_power = noise_est[i] * noise_est[i];
				double wiener_filter = signal_power / (signal_power + noise_power);
				r->fft_freq[i] *= wiener_filter;
			}
		}
  		*/

		// Noise Estimation, ANR, DSP mods W4WHL
		if (!noise_est_initialized || noise_update_counter >= noise_update_interval)
		{
			for (i = 0; i < MAX_BINS; i++)
			{
				double current_magnitude = cabs(r->fft_freq[i]);

				// Dynamically adjust noise estimation rate vs fixed
				double dynamic_alpha = (current_magnitude > noise_est[i]) ? 0.95 : 0.75;
				noise_est[i] = dynamic_alpha * noise_est[i] + (1 - dynamic_alpha) * current_magnitude;

				// Enforce a noise floor
				noise_est[i] = fmax(1e-6, noise_est[i]);
			}
			noise_update_counter = 0;
			noise_est_initialized = 1;
		}
		else
		{
			noise_update_counter++;
		}

		if (dsp_enabled)
		{
			// Spectral Subtraction filter
			for (i = 0; i < MAX_BINS; i++)
			{
				double magnitude = cabs(r->fft_freq[i]);
				double phase = carg(r->fft_freq[i]);
				double noise_magnitude = noise_est[i];

				// Calculate the SNR
				double snr = magnitude / (noise_magnitude + 1e-6); // Avoid division by zero
				double new_magnitude;

				// Sigmoid-based reduction factor
				double reduction_factor = 1.0 / (1.0 + exp(-5.0 * (snr - 0.5))); // Sharp and low-midpoint curve


				// Calculate new magnitude with residual noise preservation
				double noise_residual = 0.10; // Retain 10% of noise, reduces
				new_magnitude = fmax(noise_residual * noise_magnitude,
									magnitude - reduction_factor * noise_magnitude);

				// Smoother bin-to-bin transitions (blend current and adjacent bins)
				static double previous_magnitude[MAX_BINS] = {0};
				new_magnitude = 0.9 * new_magnitude + 0.1 * previous_magnitude[i]; // Stronger weight on current bin
				previous_magnitude[i] = new_magnitude;

				// Reconstruct the frequency domain signal
				r->fft_freq[i] = new_magnitude * cexp(I * phase);
			}
		}

		if (anr_enabled)
		{
			// Signal Estimation for Wiener filter
			for (i = 0; i < MAX_BINS; i++)
			{
				double current_magnitude = cabs(r->fft_freq[i]);
				signal_est[i] = SIGNAL_ALPHA * signal_est[i] + (1 - SIGNAL_ALPHA) * current_magnitude;
			}

			// Wiener Filter (ANR)
			for (i = 0; i < MAX_BINS; i++)
			{
				double signal_power = fmax(1e-6, signal_est[i] * signal_est[i]);
				double noise_power = fmax(1e-6, noise_est[i] * noise_est[i]);

				// Relaxed Wiener filter gain
				double wiener_filter = (signal_power + 0.2 * noise_power) / (signal_power + noise_power);
				wiener_filter = fmax(0.2, wiener_filter); // Minimum gain to preserve quiet signals

				r->fft_freq[i] *= wiener_filter;
			}

			// Improved bin smoothing
			for (i = 1; i < MAX_BINS - 1; i++)
			{
				r->fft_freq[i] = (0.8 * r->fft_freq[i]) + (0.1 * r->fft_freq[i - 1]) + (0.1 * r->fft_freq[i + 1]);
			}
		}
	}

	// STEP 5: Zero out the other sideband
	switch (r->mode)
	{
	case MODE_LSB:
	case MODE_CWR:
		for (i = 0; i < MAX_BINS / 2; i++)
		{
			__real__ r->fft_freq[i] = 0;
			__imag__ r->fft_freq[i] = 0;
		}
		break;
	case MODE_AM:
		break;
	default:
		for (i = MAX_BINS / 2; i < MAX_BINS; i++)
		{
			__real__ r->fft_freq[i] = 0;
			__imag__ r->fft_freq[i] = 0;
		}
		break;
	}

	// STEP 6: Apply the FIR filter
	for (i = 0; i < MAX_BINS; i++)
	{
		r->fft_freq[i] *= r->filter->fir_coeff[i];
	}

	// STEP 7: Convert back to time domain
	my_fftw_execute(r->plan_rev);

	// STEP 8: AGC
	agc2(r);

	// STEP 9: Send the output
	// int is_digital = 0;
	if (rx_list->output == 0)
	{
		if (r->mode == MODE_AM)
		{
			for (i = 0; i < MAX_BINS / 2; i++)
			{
				int32_t sample = cabs(r->fft_time[i + (MAX_BINS / 2)]);
				output_speaker[i] = sample;
				output_tx[i] = 0;
			}
		}
		else
		{
			int32_t sample;
			for (i = 0; i < MAX_BINS / 2; i++)
			{
				sample = cimag(r->fft_time[i + (MAX_BINS / 2)]);
				output_speaker[i] = sample;
				output_tx[i] = 0;
			}
		}

		// Push the samples to the remote audio queue, decimated to 16000 samples/sec
		//for (i = 0; i < MAX_BINS / 2; i += 6)
		//{
		//	q_write(&qremote, output_speaker[i]);
		//}
	}

	if (mute_count)
	{
		memset(output_speaker, 0, MAX_BINS / 2 * sizeof(int32_t));
		mute_count--;
	}

	// Push the data to any potential modem
	modem_rx(rx_list->mode, output_speaker, MAX_BINS / 2);

	// Apply RXEQ after Modem only on non-digital modes
	if (r->mode != MODE_DIGITAL && r->mode != MODE_FT8 && r->mode != MODE_2TONE)
{
    if (rx_eq_is_enabled == 1)
    {
        // Step 1: Apply EQ with built-in normalization and clamping
        apply_eq(&rx_eq, output_speaker, n_samples, 48000.0);

        // Step 2: Optionally apply soft limiting (only if additional smoothing is required)
        const double limiter_threshold = 0.8 * 500000000; // Lower limiter threshold for headroom

        for (int i = 0; i < n_samples; i++)
        {
            double sample = output_speaker[i];

            // Apply smooth limiting if sample exceeds threshold
            if (fabs(sample) > limiter_threshold)
            {
                sample = limiter_threshold * tanh(sample / limiter_threshold);
            }

            output_speaker[i] = (int32_t)sample;
        }
    }
}
// Push the samples to the remote audio queue, decimated to 16000 samples/sec
// Moved after EQ processing so qremote gets the equalized audio when applicable
	if (rx_list->output == 0) {
		for (i = 0; i < MAX_BINS / 2; i += 6)
		{
			q_write(&qremote, output_speaker[i]);
		}
	}
}
void read_power()
{
	uint8_t response[4];
	int16_t vfwd, vref;
	int fwdpw;

	char buff[20];

	if (!in_tx)
		return;
	if (i2cbb_read_i2c_block_data(0x8, 0, 4, response) == -1)
		return;

	vfwd = vref = 0;

	memcpy(&vfwd, response, 2);
	memcpy(&vref, response + 2, 2);
	//	printf("%d:%d\n", vfwd, vref);

	// Very low power readings may spoil the swr calculation, especially in CW modes between symbols
	// Better not to calculate the swr at all if the measured power is under a very minimal level
	if (vfwd > 3) {
		if (vref >= vfwd)
			vswr = 100;
		else
			vswr = (10 * (vfwd + vref)) / (vfwd - vref);
	}

	// here '400' is the scaling factor as our ref power output is 40 watts
	// this calculates the power as 1/10th of a watt, 400 = 40 watts
	int fwdvoltage = (vfwd * 40) / bridge_compensation;

	// Implement a simple "hold" algorithm in order to show
	// readable and meaningful power readings that should be the pep power
	fwdpw = (fwdvoltage * fwdvoltage) / 400;
	if (fwdpw > fwdpower_calc) {
		fwdpower_calc = fwdpw;
	}
	if (!fwdpower_cnt) {
		fwdpower = fwdpower_calc;
		fwdpower_calc = fwdpw;
	}
	if (!fwdpower)
		fwdpower = fwdpw;
	fwdpower_cnt = ++fwdpower_cnt % 100;

	int rf_v_p2p = (fwdvoltage * 126) / 400;
	//	printf("rf volts: %d, alc %g, %d watts ", rf_v_p2p, alc_level, fwdpower/10);
	if (rf_v_p2p > 135 && !in_calibration)
	{
		alc_level *= 135.0 / (1.0 * rf_v_p2p);
		printf("ALC tripped, to %d percent\n", (int)(100 * alc_level));
	}
	/*	else if (alc_level < 0.95){
			printf("alc releasing to ");
			alc_level *= 1.02;
		}
	*/
	//	printf("alc: %g\n", alc_level);
}

static int tx_process_restart = 0;

void tx_process(
	int32_t *input_rx, int32_t *input_mic,
	int32_t *output_speaker, int32_t *output_tx,
	int n_samples)
{
	int i;
	double i_sample, q_sample, i_carrier;

	// Check if browser microphone is active and use it instead of physical mic
	int32_t browser_mic_samples[n_samples];
	int use_browser_mic = is_browser_mic_active();

	if (use_browser_mic) {
		// Get upsampled browser mic audio
		upsample_browser_mic(browser_mic_samples, n_samples);
	}

	struct rx *r = tx_list;

	// fix the burst at the start of transmission
	if (tx_process_restart)
	{
		fft_reset_m_bins();
		tx_process_restart = 0;
	}
	static int eq_initialized = 0;

	if (!eq_initialized)
	{
		init_eq(&tx_eq, "tx");
		eq_initialized = 1;
	}

	if (in_tx && (r->mode != MODE_DIGITAL && r->mode != MODE_FT8 && r->mode != MODE_2TONE && r->mode != MODE_CW && r->mode != MODE_CWR))
	{

		// Apply compression is the value of the dial is set to 1-10 (0 = off)
		if (compression_control_level >= 1 && compression_control_level <= 10)
		{
			float temp_input_mic[n_samples];
			for (int i = 0; i < 5 && i < n_samples; i++)
			{
			}
			// Convert input_mic (int32_t) to float for compression
			for (int i = 0; i < n_samples; i++)
			{
				if (use_browser_mic) {
					temp_input_mic[i] = (float)browser_mic_samples[i] / 2000000000.0;
				} else {
					temp_input_mic[i] = (float)input_mic[i] / 2000000000.0;
				}
			}

			for (int i = 0; i < 5 && i < n_samples; i++)
			{
			}
			// Now we can call the apply_fixed_compression function with the parameters
			apply_fixed_compression(temp_input_mic, n_samples, compression_control_level);

			for (int i = 0; i < 5 && i < n_samples; i++)
			{
			}
			// Convert back the processed data to int32_t after compression
			for (int i = 0; i < n_samples; i++)
			{
				if (use_browser_mic) {
					browser_mic_samples[i] = (int32_t)(temp_input_mic[i] * 2000000000.0);
				} else {
					input_mic[i] = (int32_t)(temp_input_mic[i] * 2000000000.0);
				}
			}
			for (int i = 0; i < 5 && i < n_samples; i++)
			{
			}
		}

		if (eq_is_enabled == 1)
		{
			if (use_browser_mic) {
				apply_eq(&tx_eq, browser_mic_samples, n_samples, 48000.0);
			} else {
				apply_eq(&tx_eq, input_mic, n_samples, 48000.0);
			}
		}
	}

	if (mute_count && (r->mode == MODE_USB || r->mode == MODE_LSB || r->mode == MODE_AM))
	{
		memset(input_mic, 0, n_samples * sizeof(int32_t));
		if (use_browser_mic) {
			memset(browser_mic_samples, 0, n_samples * sizeof(int32_t));
		}
		mute_count--;
	}
	// first add the previous M samples
	for (i = 0; i < MAX_BINS / 2; i++)
		fft_in[i] = fft_m[i];

	int m = 0;
	int j = 0;

	// double max = -10.0, min = 10.0;
	// gather the samples into a time domain array
	for (i = MAX_BINS / 2; i < MAX_BINS; i++)
	{

		if (r->mode == MODE_2TONE)
			i_sample = (1.0 * (vfo_read(&tone_a) + vfo_read(&tone_b))) / 50000000000.0;
		else if (r->mode == MODE_CALIBRATE)
			i_sample = (1.0 * (vfo_read(&tone_a))) / 30000000000.0;
		else if (r->mode == MODE_CW || r->mode == MODE_CWR || r->mode == MODE_FT8)
			i_sample = modem_next_sample(r->mode) / 3;
		else if (r->mode == MODE_AM)
		{
			// double modulation = (1.0 * vfo_read(&tone_a)) / 1073741824.0;
			double modulation;
			if (use_browser_mic) {
				modulation = (1.0 * browser_mic_samples[j]) / 200000000.0;
			} else {
				modulation = (1.0 * input_mic[j]) / 200000000.0;
			}
			if (modulation < -1.0)
				modulation = -1.0;
			i_carrier = (1.0 * vfo_read(&am_carrier)) / 50000000000.0;
			i_sample = (1.0 + modulation) * i_carrier;
		}
		else
		{
			if (use_browser_mic) {
				i_sample = (1.0 * browser_mic_samples[j]) / 2000000000.0;
			} else {
				i_sample = (1.0 * input_mic[j]) / 2000000000.0;
			}
		}

		// clip the overdrive to prevent damage up the processing chain, PA
		if (r->mode == MODE_USB || r->mode == MODE_LSB || r->mode == MODE_AM)
		{
			if (i_sample < (-1.0 * voice_clip_level))
				i_sample = -1.0 * voice_clip_level;
			else if (i_sample > voice_clip_level)
				i_sample = voice_clip_level;
		}

		// Don't echo the voice modes
		if (r->mode == MODE_USB || r->mode == MODE_LSB || r->mode == MODE_AM || r->mode == MODE_NBFM)
		{
			// Unless of course you want to use the txmon control
			if (txmon_control_level >= 1 && txmon_control_level <= 10)
			{
				output_speaker[j] = i_sample * txmon_control_level * 1000000000.0;
			}
			else
			{
				output_speaker[j] = 0;
			}
			q_sample = 0;
		}
		else
		{
			// If not in voice modes, use the sidetone
			output_speaker[j] = i_sample * sidetone;
			q_sample = 0;
		}

		j++;

		__real__ fft_m[m] = i_sample;
		__imag__ fft_m[m] = q_sample;

		__real__ fft_in[i] = i_sample;
		__imag__ fft_in[i] = q_sample;
		m++;
	}

	// push the samples to the remote audio queue, decimated to 16000 samples/sec
	for (i = 0; i < MAX_BINS / 2; i += 6) {
		q_write(&qremote, output_speaker[i]);
	}

	// convert to frequency
	fftw_execute(plan_fwd);

	// NOTE: fft_out holds the fft output (in freq domain) of the
	// incoming mic samples
	// the naming is unfortunate

	// apply the filter
	for (i = 0; i < MAX_BINS; i++)
		fft_out[i] *= tx_filter->fir_coeff[i];

	// the usb extends from 0 to MAX_BINS/2 - 1,
	// the lsb extends from MAX_BINS - 1 to MAX_BINS/2 (reverse direction)
	// zero out the other sideband

	// TBD: Something strange is going on, this should have been the otherway

	if (r->mode == MODE_LSB || r->mode == MODE_CWR)
		// zero out the LSB
		for (i = 0; i < MAX_BINS / 2; i++)
		{
			__real__ fft_out[i] = 0;
			__imag__ fft_out[i] = 0;
		}
	else if (r->mode != MODE_AM)
		// zero out the USB
		for (i = MAX_BINS / 2; i < MAX_BINS; i++)
		{
			__real__ fft_out[i] = 0;
			__imag__ fft_out[i] = 0;
		}
	// adjust USB/CW modulation power factor W9JES
	for (i = 0; i < MAX_BINS / 2; i++)
	{
		__real__ fft_out[i] = __real__ fft_out[i] * ssb_val;
		__imag__ fft_out[i] = __imag__ fft_out[i] * ssb_val;
	}

	// now rotate to the tx_bin
	// rememeber the AM is already a carrier modulated at 24 KHz
	int shift = tx_shift;
	if (r->mode == MODE_AM)
		shift = 0;
	for (i = 0; i < MAX_BINS; i++)
	{
		int b = i + shift;
		if (b >= MAX_BINS)
			b = b - MAX_BINS;
		if (b < 0)
			b = b + MAX_BINS;
		r->fft_freq[b] = fft_out[i];
	}

	// the spectrum display is updated
	// spectrum_update();

	// convert back to time domain
	fftw_execute(r->plan_rev);
	int min = 10000000;
	int max = -10000000;
	float scale = volume;
	for (i = 0; i < MAX_BINS / 2; i++)
	{
		double s = creal(r->fft_time[i + (MAX_BINS / 2)]);
		output_tx[i] = s * scale * tx_amp * alc_level;
		if (min > output_tx[i])
			min = output_tx[i];
		if (max < output_tx[i])
			max = output_tx[i];
		// output_tx[i] = 0;
	}
	//	printf("min %d, max %d\n", min, max);

	read_power();

	// Instead of using sdr_modulation_update, we'll update the spectrum data directly
	// This allows the TX audio to be displayed in the spectrum and waterfall

	// Create input buffer for FFT
	complex float *tx_fft_in = (complex float *)malloc(sizeof(complex float) * MAX_BINS);

	// Calculate DC offset (average) to remove it
	float dc_offset = 0;
	for (i = 0; i < MAX_BINS / 2; i++) {
		dc_offset += output_tx[i];
	}
	dc_offset /= (MAX_BINS / 2);

	// Copy the output_tx samples to the FFT input buffer with a window function
	for (i = 0; i < MAX_BINS / 2; i++) {
		// Apply Hann window for better spectral resolution
		float window = 0.5 * (1 - cos(2 * M_PI * i / (MAX_BINS / 2 - 1)));
		// Remove DC offset and scale down
		tx_fft_in[i] = (output_tx[i] - dc_offset) * window / (tx_amp * 150000000.0); // Significantly reduced scaling
	}

	// Zero-pad the second half
	for (i = MAX_BINS / 2; i < MAX_BINS; i++) {
		tx_fft_in[i] = 0;
	}

	// Use the existing FFT infrastructure
	for (i = 0; i < MAX_BINS; i++) {
		__real__ fft_in[i] = crealf(tx_fft_in[i]);
		__imag__ fft_in[i] = 0;
	}

	// Perform FFT using the existing plan
	fftw_execute(plan_fwd);

	// Update the fft_spectrum array with the FFT results
	// This is important because the spectrum_update function uses this array

	// First pass - enhanced detail with frequency-dependent scaling
	for (i = 0; i < MAX_BINS; i++) {
		// Calculate bin frequency relative to center (for frequency-dependent scaling)
		int bin_from_center = i - MAX_BINS / 2;
		if (bin_from_center < 0) bin_from_center = -bin_from_center;

		// Apply slightly higher gain to mid-range frequencies where voice details matter most
		float freq_scale = 1.0;
		if (bin_from_center > 10 && bin_from_center < 100) {
			freq_scale = 1.3; // Boost mid-range frequencies
		}

		// Store the FFT results with enhanced detail
		fft_spectrum[i] = fft_out[i] * 0.025 * freq_scale; // Slightly increased from 0.02 for more detail
	}

	// Apply a more refined smoothing that preserves detail while reducing noise
	complex float *smoothed = (complex float *)malloc(sizeof(complex float) * MAX_BINS);

	// Copy first and last points as-is
	smoothed[0] = fft_spectrum[0];
	smoothed[MAX_BINS-1] = fft_spectrum[MAX_BINS-1];

	// Apply minimal smoothing to preserve maximum detail
	for (i = 1; i < MAX_BINS-1; i++) {
		// Use weighted average with heavy weight on current bin: 80% current bin, 10% each adjacent bin
		smoothed[i] = fft_spectrum[i-1] * 0.1 + fft_spectrum[i] * 0.8 + fft_spectrum[i+1] * 0.1;

		// Apply stronger contrast enhancement to make fine details more visible
		float mag = cabsf(smoothed[i]);
		if (mag > 0) {
			// Use stronger non-linear enhancement to reveal subtle details
			smoothed[i] *= (1.0 + 0.5 * log10f(mag + 1.0));

			// Add slight sharpening effect to enhance edges between frequency components
			if (i > 1 && i < MAX_BINS-2) {
				complex float edge_detect = smoothed[i] * 2.0 - smoothed[i-1] * 0.5 - smoothed[i+1] * 0.5;
				smoothed[i] = smoothed[i] * 0.7 + edge_detect * 0.3;
			}
		}
	}

	// Copy smoothed spectrum back to fft_spectrum
	for (i = 0; i < MAX_BINS; i++) {
		fft_spectrum[i] = smoothed[i];
	}

	// Free the temporary buffer
	free(smoothed);

	// Call the standard spectrum update function to ensure consistent processing
	spectrum_update();

	// Clean up
	free(tx_fft_in);

	// The old sdr_modulation_update function is still called for API compatibility
	sdr_modulation_update(output_tx, MAX_BINS / 2, tx_amp);
}

/*
	This is called each time there is a block of signal samples ready
	either from the mic or from the rx IF
*/

void sound_process(
	int32_t *input_rx, int32_t *input_mic,
	int32_t *output_speaker, int32_t *output_tx,
	int n_samples)
{
	if (in_tx)
	{
		tx_process(input_rx, input_mic, output_speaker, output_tx, n_samples);
	}
	else
	{
		rx_linear(input_rx, input_mic, output_speaker, output_tx, n_samples);
	}

	if (pf_record)
	{
		wav_record(in_tx == 0 ? output_speaker : input_mic, n_samples);
	}
}

// Existing set_rx_filter function
void set_rx_filter()
{
	// on AM filter at the IF level, instead of the baseband
	if (rx_list->mode == MODE_AM)
	{
		printf("Setting AM filter\n");
		filter_tune(rx_list->filter,
					(1.0 * (24000 - rx_list->high_hz)) / 96000.0,
					(1.0 * (24000 + rx_list->high_hz)) / 96000.0,
					5);
	}
	else if (rx_list->mode == MODE_LSB || rx_list->mode == MODE_CWR)
	{
		filter_tune(rx_list->filter,
					(1.0 * -rx_list->high_hz) / 96000.0,
					(1.0 * -rx_list->low_hz) / 96000.0,
					5);
	}
	else
	{
		filter_tune(rx_list->filter,
					(1.0 * rx_list->low_hz) / 96000.0,
					(1.0 * rx_list->high_hz) / 96000.0,
					5);
	}
}

/*
Write code that mus repeatedly so things, it is called during the idle time
of the event loop
*/
void loop()
{
	delay(10);
}

void signal_handler(int signum)
{
	digitalWrite(TX_LINE, LOW);
}

void setup_audio_codec()
{
	strcpy(audio_card, "hw:0");

	// configure all the channels of the mixer
	sound_mixer(audio_card, "Input Mux", 0);
	sound_mixer(audio_card, "Line", 1);
	sound_mixer(audio_card, "Mic", 0);
	sound_mixer(audio_card, "Mic Boost", 0);
	sound_mixer(audio_card, "Playback Deemphasis", 0);

	sound_mixer(audio_card, "Master", 10);
	sound_mixer(audio_card, "Output Mixer HiFi", 1);
	sound_mixer(audio_card, "Output Mixer Mic Sidetone", 0);
}

void setup_oscillators()
{
	// initialize the SI5351

	delay(200);
	si5351bx_init();
	delay(200);
	si5351bx_setfreq(1, bfo_freq + bfo_freq_runtime_offset);
	// TODO:  Seems to not be needed, but worth testing before release - n1qm
	// delay(200);
	// si5351bx_setfreq(1, bfo_freq + bfo_freq_runtime_offset);

	si5351_reset();
}
void resetup_oscillators()
{
	// re-initialize the SI5351
	si5351bx_setfreq(1, bfo_freq + bfo_freq_runtime_offset);
	si5351_reset();
}
static int hw_init_index = 0;
static int hw_settings_handler(void *user, const char *section,
							   const char *name, const char *value)
{
	char cmd[1000];
	char new_value[200];

	if (!strcmp(name, "f_start"))
		band_power[hw_init_index].f_start = atoi(value);
	if (!strcmp(name, "f_stop"))
		band_power[hw_init_index].f_stop = atoi(value);
	if (!strcmp(name, "scale"))
		band_power[hw_init_index++].scale = atof(value);
	if (!strcmp(name, "bfo_freq"))
		bfo_freq = atoi(value);
	// Add variable for SSB/CW Power Factor Adjustment W9JES
	if (!strcmp(name, "ssb_val"))
		ssb_val = atof(value);
	// Add TCXO Calibration W9JES/KK4DAS
	if (!strcmp(section, "tcxo"))
	{
		if (!strcmp(name, "cal"))
		{
			//  printf("xtal_freq_cal = %d\n",atoi(value));
			si5351_set_calibration(atoi(value));
		}
	}
}

static void read_hw_ini()
{
	hw_init_index = 0;
	char directory[PATH_MAX];
	char *path = getenv("HOME");
	strcpy(directory, path);
	strcat(directory, "/sbitx/data/hw_settings.ini");
	if (ini_parse(directory, hw_settings_handler, NULL) < 0)
	{
		printf("Unable to load ~/sbitx/data/hw_settings.ini\nLoading default_hw_settings.ini instead\n");
		strcpy(directory, path);
		strcat(directory, "/sbitx/data/default_hw_settings.ini");
		ini_parse(directory, hw_settings_handler, NULL);
	}
}

/*
	 the PA gain varies across the band from 3.5 MHz to 30 MHz
	here we adjust the drive levels to keep it up, almost level
*/
void set_tx_power_levels()
{
	 printf("Setting tx_power drive to %d\n", tx_drive);
	// int tx_power_gain = 0;

	// search for power in the approved bands
	for (int i = 0; i < sizeof(band_power) / sizeof(struct power_settings); i++)
	{
		if (band_power[i].f_start <= freq_hdr && freq_hdr <= band_power[i].f_stop)
		{

			// next we do a decimal coversion of the power reduction needed
			tx_amp = (1.0 * tx_drive * band_power[i].scale);
		}
	}
	//	printf("tx_amp is set to %g for %d drive\n", tx_amp, tx_drive);
	// we keep the audio card output 'volume' constant'
	sound_mixer(audio_card, "Master", 95);
	sound_mixer(audio_card, "Capture", tx_gain);
	alc_level = 1.0;
}

/* calibrate power on all bands */

void calibrate_band_power(struct power_settings *b)
{

	set_rx1(b->f_start + 35000);
	printf("*calibrating for %d\n", freq_hdr);
	tx_list->mode = MODE_CALIBRATE;
	tx_drive = 100;

	int i, j;

	//	double scale_delta = b->scale / 25;
	double scaling_factor = 0.0001;
	b->scale = scaling_factor;
	set_tx_power_levels();
	delay(50);

	tr_switch(1);
	delay(100);

	for (i = 0; i < 200 && b->scale < 0.015; i++)
	{
		scaling_factor *= 1.1;
		b->scale = scaling_factor;
		set_tx_power_levels();
		delay(50); // let the new power levels take hold

		int avg = 0;
		// take many readings to get a peak
		for (j = 0; j < 10; j++)
		{
			delay(20);
			avg += fwdpower / 10; // fwdpower in 1/10th of a watt
								  //			printf("  avg %d, fwd %d scale %g\n", avg, fwdpower, b->scale);
		}
		avg /= 10;
		printf("*%d, f %d : avg %d, max = %d\n", i, b->f_start, avg, b->max_watts);
		if (avg >= b->max_watts)
			break;
	}
	tr_switch(0);
	printf("*tx scale for %d is set to %g\n", b->f_start, b->scale);
	delay(100);
}

static void save_hw_settings()
{
	static int last_save_at = 0;
	char file_path[200]; // dangerous, find the MAX_PATH and replace 200 with it

	char *path = getenv("HOME");
	strcpy(file_path, path);
	strcat(file_path, "/sbitx/data/hw_settings.ini");

	FILE *f = fopen(file_path, "w");
	if (!f)
	{
		printf("Unable to save %s : %s\n", file_path, strerror(errno));
		return;
	}

	fprintf(f, "bfo_freq=%d\n\n", bfo_freq);
	// now save the band stack
	for (int i = 0; i < sizeof(band_power) / sizeof(struct power_settings); i++)
	{
		fprintf(f, "[tx_band]\nf_start=%d\nf_stop=%d\nscale=%g\n\n",
				band_power[i].f_start, band_power[i].f_stop, band_power[i].scale);
	}

	fclose(f);
}

pthread_t calibration_thread;

void *calibration_thread_function(void *server)
{

	int old_freq = freq_hdr;
	int old_mode = tx_list->mode;
	int old_tx_drive = tx_drive;

	in_calibration = 1;
	for (int i = 0; i < sizeof(band_power) / sizeof(struct power_settings); i++)
	{
		calibrate_band_power(band_power + i);
	}
	in_calibration = 0;

	set_rx1(old_freq);
	tx_list->mode = old_mode;
	tx_drive = old_tx_drive;
	save_hw_settings();
	printf("*Finished band power calibrations\n");
}

void tx_cal()
{
	printf("*Starting tx calibration, with dummy load connected\n");
	pthread_create(&calibration_thread, NULL, calibration_thread_function,
				   (void *)NULL);
}

// tr_switch replaces separate tr_switch_de and tr_switch_v2
// added and edited comments
// removed several delay() calls
// eliminated LPF switching during tr_switch

void tr_switch_de(int tx_on) {
  // function replaced by tr_switch, should never be called
}

void tr_switch_v2(int tx_on) {
  // function replaced by tr_switch, should never be called
}

// transmit-receive switch for both sbitx DE and V2 and newer
void tr_switch(int tx_on) {
  if (tx_on) {                   // switch to transmit
    in_tx = 1;                   // raise a flag so functions see we are in transmit mode
    sound_mixer(audio_card, "Master", 0);  // mute audio while switching to transmit
    sound_mixer(audio_card, "Capture", 0);
		if (rx_list->mode != MODE_CW && rx_list->mode != MODE_CWR) {
		delay(20);
	}
    //mute_count = 20;             // number of audio samples to zero out
    mute_count = 1;             // number of audio samples to zero out
	fft_reset_m_bins();          // fixes burst at start of transmission
    set_tx_power_levels();       // use values for tx_power_watts, tx_gain
    //ADDED BY KF7YDU - Check if ptt is enabled, if so, set ptt pin to high
			if (ext_ptt_enable == 1) {
				digitalWrite(EXT_PTT, HIGH);
				delay(20); //this delay gives time for ext device to settle before tx
			}
    digitalWrite(TX_LINE, HIGH);  // power up PA and disconnect receiver
    spectrum_reset();

    // Also reset the hold counter for showing the output power
    fwdpower_cnt = 0;
    fwdpower_calc = 0;
    fwdpower = 0;

  } else {                       // switch to receive
    in_tx = 0;                   // lower the transmit flag
    sound_mixer(audio_card, "Master", 0);  // mute audio while switching to receive
    sound_mixer(audio_card, "Capture", 0);
    fft_reset_m_bins();
    mute_count = MUTE_MAX;
    digitalWrite(EXT_PTT, LOW);  // added by KF7YDU - shuts down ext_ptt
    delay(5);
    digitalWrite(TX_LINE, LOW);  // use T/R switch to connect rcvr
    check_r1_volume();           // audio codec is back on
    initialize_rx_vol();         // added to set volume after tx -W2JON W9JES KB2ML
    sound_mixer(audio_card, "Master", rx_vol);
    sound_mixer(audio_card, "Capture", rx_gain);
    spectrum_reset();

    // Also reset the hold counter for showing the output power
    fwdpower_cnt = 0;
    fwdpower_calc = 0;
    fwdpower = 0;
  }
}

/*
This is the one-time initialization code
*/
void setup()
{

	read_hw_ini();

	// create_mcast_socket();

	// setup the LPF and the gpio pins
	pinMode(TX_LINE, OUTPUT);
	pinMode(TX_POWER, OUTPUT);
	pinMode(EXT_PTT, OUTPUT); // ADDED BY KF7YDU
	pinMode(LPF_A, OUTPUT);
	pinMode(LPF_B, OUTPUT);
	pinMode(LPF_C, OUTPUT);
	pinMode(LPF_D, OUTPUT);
	digitalWrite(LPF_A, LOW);
	digitalWrite(LPF_B, LOW);
	digitalWrite(LPF_C, LOW);
	digitalWrite(LPF_D, LOW);

	// ADDED BY KF7YDU - initialize ext_ptt to low at startup
	digitalWrite(EXT_PTT, LOW);

	digitalWrite(TX_LINE, LOW);
	digitalWrite(TX_POWER, LOW);

	fft_init();
	vfo_init_phase_table();
	setup_oscillators();
	//initialize the queues
	q_init(&qremote, 8000);
	q_init(&qbrowser_mic, 32000); // Initialize browser microphone queue with much larger buffer

	// Initialize jitter buffer
	jitter_buffer_write = 0;
	jitter_buffer_read = 0;
	jitter_buffer_samples = 0;

	modem_init();

	add_rx(7000000, MODE_LSB, -3000, -300);
	add_tx(7000000, MODE_LSB, -3000, -300);
	rx_list->tuned_bin = 512;
	tx_list->tuned_bin = 512;
	tx_init(7000000, MODE_LSB, -3000, -150);

	// detect the version of sbitx
	uint8_t response[4];
	if (i2cbb_read_i2c_block_data(0x8, 0, 4, response) == -1)
		sbitx_version = SBITX_DE;
	else
		sbitx_version = SBITX_V2;

	setup_audio_codec();
	sound_thread_start("plughw:0,0");

	sleep(1); // why? to allow the aloop to initialize?

	vfo_start(&tone_a, 700, 0);
	vfo_start(&tone_b, 1900, 0);
	vfo_start(&am_carrier, 24000, 0);
	delay(2000);
	//	pf_debug = fopen("am_test.raw", "w");
}
void sdr_request(char *request, char *response)
{
	char cmd[100], value[1000];

	char *p = strchr(request, '=');
	int n = p - request;
	if (!p)
		return;
	strncpy(cmd, request, n);
	cmd[n] = 0;
	strcpy(value, request + n + 1);

	if (!strcmp(cmd, "stat:tx"))
	{
		if (in_tx)
			strcpy(response, "ok on");
		else
			strcpy(response, "ok off");
	}
	else if (!strcmp(cmd, "r1:freq"))
	{
		int d = atoi(value);
		set_rx1(d);
		// printf("Frequency set to %d\n", freq_hdr);
		strcpy(response, "ok");
	}
	else if (!strcmp(cmd, "r1:mode"))
	{
		if (!strcmp(value, "LSB"))
			rx_list->mode = MODE_LSB;
		else if (!strcmp(value, "CW"))
			rx_list->mode = MODE_CW;
		else if (!strcmp(value, "CWR"))
			rx_list->mode = MODE_CWR;
		else if (!strcmp(value, "2TONE"))
			rx_list->mode = MODE_2TONE;
		else if (!strcmp(value, "TUNE")) // W9JES
			rx_list->mode = MODE_CALIBRATE;
		else if (!strcmp(value, "FT8"))
			rx_list->mode = MODE_FT8;
		else if (!strcmp(value, "AM"))
			rx_list->mode = MODE_AM;
		else if (!strcmp(value, "DIGI"))
			rx_list->mode = MODE_DIGITAL;
		else
			rx_list->mode = MODE_USB;

		// set the tx mode to that of the rx1
		tx_list->mode = rx_list->mode;

		// An interesting but non-essential note:
		// the sidebands inverted twice, to come out correctly after all
		// conisder that the second oscillator is set to 27.025 MHz and
		// a 7 MHz signal is tuned in by a 34 Mhz oscillator.
		// The first IF will be 25 Mhz, converted to a second IF of 25 KHz
		// Now, imagine that the signal at 7 Mhz moves up by 1 Khz
		// the IF now is going to be 34 - 7.001 MHz = 26.999 MHz which
		// converts to a second IF of 26.999 - 27.025 = 26 KHz
		// Effectively, if a signal moves up, so does the second IF

		if (rx_list->mode == MODE_AM)
		{
			// puts("\n\n\ntx am filter ");
			filter_tune(tx_list->filter,
						(1.0 * 19000) / 96000.0,
						(1.0 * 29000) / 96000.0,
						5);
			filter_tune(tx_filter,
						(1.0 * 19000) / 96000.0,
						(1.0 * 29000) / 96000.0,
						5);
		}

		else if (rx_list->mode == MODE_LSB || rx_list->mode == MODE_CWR)
		{
			// puts("\n\n\ntx LSB filter ");
			filter_tune(tx_list->filter,
						(1.0 * -3500) / 96000.0,
						(1.0 * -100) / 96000.0,
						5);
			filter_tune(tx_filter,
						(1.0 * -3500) / 96000.0,
						(1.0 * -100) / 96000.0,
						5);
		}
		else
		{
			// puts("\n\n\ntx USB filter ");
			filter_tune(tx_list->filter,
						(1.0 * 300) / 96000.0,
						(1.0 * 3500) / 96000.0,
						5);
			filter_tune(tx_filter,
						(1.0 * 300) / 96000.0,
						(1.0 * 3500) / 96000.0,
						5);
		}

		// we need to nudge the oscillator to adjust
		// to cw offset. setting it to the already tuned freq
		// doesnt recalculte the offsets

		int f = freq_hdr;
		set_rx1(f - 10);
		set_rx1(f);

		// printf("mode set to %d\n", rx_list->mode);
		strcpy(response, "ok");
	}
	else if (!strcmp(cmd, "txmode"))
	{
		puts("\n\n\n\n###### tx filter #######");
		if (!strcmp(value, "LSB") || !strcmp(value, "CWR"))
			filter_tune(tx_filter, (1.0 * -3000) / 96000.0, (1.0 * -300) / 96000.0, 5);
		else
			filter_tune(tx_filter, (1.0 * 300) / 96000.0, (1.0 * 3000) / 96000.0, 5);
	}
	else if (!strcmp(cmd, "record"))
	{
		if (!strcmp(value, "off"))
		{
			fclose(pf_record);
			pf_record = NULL;
		}
		else
			pf_record = wav_start_writing(value);
	}
	else if (!strcmp(cmd, "tx"))
	{
		if (!strcmp(value, "on"))
			tr_switch(1); // W9JES KB2ML
		else
			tr_switch(0);
		strcpy(response, "ok"); // W9LES KB2ML
	}
	else if (!strcmp(cmd, "rx_pitch"))
	{
		rx_pitch = atoi(value);
	}
	else if (!strcmp(cmd, "tx_gain"))
	{
		tx_gain = atoi(value);
		if (in_tx)
			set_tx_power_levels();
	}
	else if (!strcmp(cmd, "tx_power"))
	{
		tx_drive = atoi(value);
		if (in_tx)
			set_tx_power_levels();
	}
	else if (!strcmp(cmd, "bridge"))
	{
		bridge_compensation = atoi(value);
	}
	else if (!strcmp(cmd, "r1:gain"))
	{
		rx_gain = atoi(value);
		if (!in_tx)
			sound_mixer(audio_card, "Capture", rx_gain);
	}
	// Volume control scaling adjustment - W2JON
	else if (!strcmp(cmd, "r1:volume"))
	{
		int input_volume = atoi(value);
		int rx_vol;

		if (input_volume == 0)
		{
			rx_vol = 0;
		}
		else
		{
			rx_vol = (int)(log10(1 + 9 * input_volume) * 100 / log10(1 + 900));
		}

		if (!in_tx)
		{
			sound_mixer(audio_card, "Master", rx_vol);
		}
	}
	//
	else if (!strcmp(cmd, "r1:high"))
	{
		rx_list->high_hz = atoi(value);
		set_rx_filter();
	}
	else if (!strcmp(cmd, "r1:low"))
	{
		rx_list->low_hz = atoi(value);
		set_rx_filter();
	}
	else if (!strcmp(cmd, "r1:agc"))
	{
		if (!strcmp(value, "OFF"))
			rx_list->agc_speed = -1;
		else if (!strcmp(value, "SLOW"))
			rx_list->agc_speed = 100;
		else if (!strcmp(value, "MED"))
			rx_list->agc_speed = 33;
		else if (!strcmp(value, "FAST"))
			rx_list->agc_speed = 10;
	}
	else if (!strcmp(cmd, "sidetone"))
	{ // between 100 and 0
		float t_sidetone = atof(value);
		if (0 <= t_sidetone && t_sidetone <= 100)
			sidetone = atof(value) * 20000000;
	}
	else if (!strcmp(cmd, "mod"))
	{
		if (!strcmp(value, "MIC"))
			tx_use_line = 0;
		else if (!strcmp(value, "LINE"))
			tx_use_line = 1;
	}
	else if (!strcmp(cmd, "txcal"))
		tx_cal();
	else if (!strcmp(cmd, "tx_compress"))
	{
		tx_compress = atoi(value);
	}
	else if (!strcmp(cmd, "bandscale+"))
	{
		band_power[bandtweak].scale += .00025;
		printf("Band %i scale now at %f\n", band_power[bandtweak].f_start, band_power[bandtweak].scale);
	}
	else if (!strcmp(cmd, "bandscale-"))
	{
		band_power[bandtweak].scale -= .00025;
		printf("Band %i scale now at %f\n", band_power[bandtweak].f_start, band_power[bandtweak].scale);
	}
	else if (!strcmp(cmd, "adjustbsband"))
	{
		bandtweak = atoi(value);
		printf("Now adjusting band %i scale is currently: %f\n", band_power[bandtweak].f_start, band_power[bandtweak].scale);
	}

	/* else
		  printf("*Error request[%s] not accepted\n", request); */
}
