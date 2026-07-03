/*
1. The sound system is kickstarted by calling sound_thread_start() with the 
device id (as a string).

2. The sound system is run in separate thread and it keeps calling sound_process()
WARNING: sound_process() is being called from a different thread. It should
return quickly before the next set of audio data is due.

3. The left channel is used for rx and the right channel is used for tx.
The left channel takes its input (between 0 and 48 KHz( from the rx, 
demodulates it and writes out to the speaker/audio output.

4. The right channel gets audio data from mic, modulates it as a signal between
0 and 48 KHz and sends it out to right channel output.

5. A number of settings for the sound card like gain, etc can be set by calling
sound_mixer(). search for this function to know how to work this.

*/
int sound_thread_start(char *device);
void sound_process(
	int32_t *input_rx, int32_t *input_mic, 
	int32_t *output_speaker, int32_t *output_tx, 
	int n_samples);
void sound_thread_stop();
void sound_volume(char *card_name, char *element, int volume);
void sound_mixer(char *card_name, char *element, int make_on);
void sound_input(int loop);
unsigned long sbitx_millis();

//volume control normalizer
extern int input_volume;
//void set_input_volume(int volume);
int get_input_volume(void);
void check_r1_volume();


//Notch Filter
extern int notch_enabled;
extern double notch_freq; 
extern double notch_bandwidth; 

//ANR (automatic noise reduction)
extern int anr_enabled;

//rx DSP tool
extern int dsp_enabled;
extern int noise_threshold;
extern int noise_update_interval; 
double scaleNoiseThreshold(int control);

// Aduio Compression tool
extern int compression_control_level;
void apply_fixed_compression(float *input, int num_samples, int compression_control_value);

// TX Monitor tool
extern int txmon_control_level;

// Audio device name globals -- set before sound_thread_start() or sound_restart()
// All three default to the original hardcoded values so existing setups
// work without any user_settings.ini changes.
extern char pcm_device_name[64];         // main PCM capture + playback (WM8731)
extern char loopback_play_device[64];    // loopback play  (to fldigi / digimodes)
extern char loopback_capture_device[64]; // loopback capture (from fldigi / digimodes)
// Optional USB audio devices.  Leave empty string to use WM8731 only.
extern char usb_audio_play_device[64];   // USB audio out speaker (48kHz)
extern char usb_audio_cap_device[64];    // USB audio mic input (48kHz)
// Set USB playback volume (0-100); derives hw: card from plughw: device name
void sound_usb_set_volume(const char *plughw_device, int volume_pct);
// Set USB capture (mic) gain (0-100); same card derivation as above
void sound_usb_set_capture(const char *plughw_device, int gain_pct);
// Enable (1) or disable (0) the USB mic capture switch at hardware level.
// Call with enable=0 in non-voice modes to fully silence the USB mic.
void sound_usb_enable_capture(const char *plughw_device, int enable);
// Scan ALSA and fill out_play/out_cap with the first USB audio device found.
// Returns 1 if found, 0 if no USB audio hardware is present.
int  sound_find_usb_audio(char *out_play, char *out_cap, int maxlen);
// Probe whether a named ALSA PCM playback device is actually present.
// Returns 1 if the device opens successfully, 0 if not (e.g. unplugged).
int  sound_usb_device_present(const char *device);
