#include <stdio.h>
#include <ctype.h>
#include <alsa/asoundlib.h>
#include <pthread.h>
#include <complex.h>
#include <fftw3.h>
#include <sys/time.h>
#include <time.h>
#include "sound.h"
#include "wiringPi.h"
#include "sdr.h"

// Set the DEBUG define to 1 to compile in the debugging messages.
// Set the DEBUG define to 2 to compile in detailed error reporting debugging messages.
#define DEBUG -1

// Set the DISABLE_LOOPBACK define to 1 to disable the loopback processing.
#define DISABLE_LOOPBACK 0

/* follows the tutorial at http://alsamodular.sourceforge.net/alsa_programming_howto.html
Next thing to try is http://www.saunalahti.fi/~s7l/blog/2005/08/21/Full%20Duplex%20ALSA

	We are using 4 bytes per sample, 
	each frame is consists of two channels of audio, hence 8 bytes 
  We are shooting for 1024x2 = 2048 samples per period. that is 8K
  At two periods in the buffer, the buffer has to be 16K

	To simply the work, we are picking up some settings for the Wolfson codec
	as it connects to a raspberry pi. These values are interdependent
	and they will work out of the box. It takes the guess work out of
	configuring the Raspberry Pi with Wolfson codec.
*/

/*
	The audio channels are:
	PCM Playback
	Loopback Capture
	PCM Capture
	Loopback Play
*/

/*
	MIXER api

	https://alsa.opensrc.org/HowTo_access_a_mixer_control

https://android.googlesource.com/platform/hardware/qcom/audio/+/jb-mr1-dev/alsa_sound/ALSAMixer.cpp

https://github.com/bear24rw/alsa-utils/blob/master/amixer/amixer.c

There are six kinds of controls:
	playback volume 
	playback switch
	playback enumeration
	capture volume
	capture switch
	capture enumeration

examples of using amixer to mute and unmute:
amixer -c 1  set 'Output Mixer Mic Sidetone' unmute
amixer -c 1  set 'Output Mixer Mic Sidetone' mute


examples of using sound_mixer function:
'Mic' 0/1 = mute/unmute the mic
'Line' 0/1= mute/unmute the line in
'Master' 0-100 controls the earphone volume only, line out remains unaffected
'Input Mux' 1/0 take the input either from the Mic or Line In


*/

void sound_volume(char *card_name, char *element, int volume)
{
    long min, max;
    snd_mixer_t *handle;
    snd_mixer_selem_id_t *sid;
		char *card;

		card = card_name;
    snd_mixer_open(&handle, 0);
    snd_mixer_attach(handle, card);
    snd_mixer_selem_register(handle, NULL, NULL);
    snd_mixer_load(handle);

    snd_mixer_selem_id_alloca(&sid);
    snd_mixer_selem_id_set_index(sid, 0);
    snd_mixer_selem_id_set_name(sid, element);
    snd_mixer_elem_t* elem = snd_mixer_find_selem(handle, sid);

    snd_mixer_selem_get_playback_volume_range(elem, &min, &max);
    snd_mixer_selem_set_playback_volume_all(elem, volume * max / 100);

    snd_mixer_close(handle);
}

void sound_mixer(char *card_name, char *element, int make_on)
{
    long min, max;
    snd_mixer_t *handle;
    snd_mixer_selem_id_t *sid;
    char *card = card_name;

    snd_mixer_open(&handle, 0);
    snd_mixer_attach(handle, card);
    snd_mixer_selem_register(handle, NULL, NULL);
    snd_mixer_load(handle);

    snd_mixer_selem_id_alloca(&sid);
    snd_mixer_selem_id_set_index(sid, 0);
    snd_mixer_selem_id_set_name(sid, element);
    snd_mixer_elem_t* elem = snd_mixer_find_selem(handle, sid);

    /* Guard against a missing element (e.g. card present but element name
       doesn't exist, or card wasn't found at all).  Without this check,
       snd_mixer_selem_has_capture_switch(NULL) triggers an assertion abort. */
    if (!elem) {
        snd_mixer_close(handle);
        return;
    }

    //find out if the his element is capture side or plaback
    if(snd_mixer_selem_has_capture_switch(elem)){
			//puts("this is a capture switch.");  
	  	snd_mixer_selem_set_capture_switch_all(elem, make_on);
		}
    else if (snd_mixer_selem_has_playback_switch(elem)){
		//	puts("this is a playback switch.");
			snd_mixer_selem_set_playback_switch_all(elem, make_on);
		}
    else if (snd_mixer_selem_has_playback_volume(elem)){
			//puts("this is  playback volume");
			long volume = make_on;
    	snd_mixer_selem_get_playback_volume_range(elem, &min, &max);
    	snd_mixer_selem_set_playback_volume_all(elem, volume * max / 100);
    }	
    else if (snd_mixer_selem_has_capture_volume(elem)){
		//	puts("this is a capture volume");
			long volume = make_on;
    	snd_mixer_selem_get_capture_volume_range(elem, &min, &max);
    	snd_mixer_selem_set_capture_volume_all(elem, volume * max / 100);
    }
		else if (snd_mixer_selem_is_enumerated(elem)){
//			puts("TBD: this is an enumerated capture element");
			snd_mixer_selem_set_enum_item(elem, 0, make_on);
		}
    snd_mixer_close(handle);
}

int rate = 96000; /* Sample rate */
static snd_pcm_uframes_t buff_size = 8192; /* Periodsize (bytes) */ 
static int n_periods_per_buffer = 2;       /* Number of periods */
//static int n_periods_per_buffer = 1024;       /* Number of periods */

static snd_pcm_t *pcm_play_handle=0;   	//handle for the pcm device
static snd_pcm_t *pcm_capture_handle=0;   	//handle for the pcm device
static snd_pcm_t *loopback_play_handle=0;       //handle for the pcm device
static snd_pcm_t *loopback_capture_handle=0;    //handle for the pcm device
static snd_pcm_t *usb_audio_play_handle=0;      //optional USB audio out speaker
static snd_pcm_t *usb_audio_cap_handle=0;       //optional USB audio mic

static snd_pcm_stream_t play_stream = SND_PCM_STREAM_PLAYBACK;	//playback stream
static snd_pcm_stream_t capture_stream = SND_PCM_STREAM_CAPTURE;	//playback stream

static snd_pcm_hw_params_t *hwparams;
static snd_pcm_sw_params_t *swparams;
static snd_pcm_hw_params_t *hloop_params;
static snd_pcm_sw_params_t *sloop_params;
static int exact_rate;   /* Sample rate returned by */
static int	sound_thread_continue = 0;
pthread_t sound_thread, loopback_thread;

static long int last_loopback_reset = 0;		//time interval of last loopback play reset - n1qm
static int reset_loopback_interval = 300;  		// Seconds to reset loopback device

#define LOOPBACK_LEVEL_DIVISOR 8				// Constant used to reduce audio level to the loopback channel (FLDIGI)
static int pcm_capture_error = 0;				// count pcm capture errors
static int pcm_play_write_error = 0;			// count play channel write errors
static int pcm_loopback_write_error = 0;		// count loopback channel write errors
static int result = 0;							// scratch variable for storing function call results
// Note: Error messages appear when the sbitx program is started from the command line

int use_virtual_cable = 0;

// Writable device names -- override before calling sound_thread_start() or
// sound_restart().  Defaults match the original hardcoded values so that
// existing installations without user_settings.ini entries keep working.
char pcm_device_name[64]        = "plughw:CARD=audioinjectorpi,DEV=0";
char loopback_play_device[64]   = "plughw:CARD=1,DEV=0";
char loopback_capture_device[64]= "plughw:CARD=2,DEV=1";
// Optional USB audio device -- empty string means "not configured, use WM8731 only"
char usb_audio_play_device[64]  = "";
char usb_audio_cap_device[64]   = "";
unsigned long sound_millis = 0;

struct Queue qloop;

/* this function should be called just once in the application process.
Calling it frequently will result in more allocation of hw_params memory blocks
without releasing them.
The list of PCM devices available on any platform can be found by running
	aplay -L 
We have to pass the id of one of those devices to this function.
The sequence of the alsa functions must be maintained for this to work consistently

It returns a -1 if the device didn't open. The error message is on stderr.

IMPORTANT:
The sound is playback is carried on in a non-blocking way
Update - Sound playback now uses blocking ALSA calls, although the calls
do not block until the buffers are completely filled (which will never occur)

*/

int sound_start_play(char *device){
	//found out the correct device through aplay -L (for pcm devices)
	//puts a playback handle into the pointer to the pointer
	
#if DEBUG > 0	
	printf ("opening audio playback stream to %s\n", device); 
#endif
	int e = snd_pcm_open(&pcm_play_handle, device, play_stream, 0);  // was SND_PCM_NONBLOCK
	
	if (e < 0) {
		fprintf(stderr, "Error opening PCM playback device %s: %s\n", device, snd_strerror(e));
		return -1;
	}

	snd_pcm_hw_params_alloca(&hwparams);	// more alloc
	snd_pcm_sw_params_alloca(&swparams);	// more alloc

	//fills up the hwparams with values, hwparams was allocated above
	e = snd_pcm_hw_params_any(pcm_play_handle, hwparams);

	if (e < 0) {
		fprintf(stderr, "*Error getting hw playback params (%d)\n", e);
		return(-1);
	}
	
	// set the pcm access to interleaved
	e = snd_pcm_hw_params_set_access(pcm_play_handle, hwparams, SND_PCM_ACCESS_RW_INTERLEAVED);
	if (e < 0) {
		fprintf(stderr, "*Error setting playback access.\n");
		return(-1);
	}

  /* Set sample format */
	e = snd_pcm_hw_params_set_format(pcm_play_handle, hwparams, SND_PCM_FORMAT_S32_LE);
	if (e < 0) {
		fprintf(stderr, "*Error setting plyaback format.\n");
		return(-1);
	}


	/* Set sample rate. If the exact rate is not supported */
	/* by the hardware, use nearest possible rate.         */ 
	exact_rate = rate;
	e = snd_pcm_hw_params_set_rate(pcm_play_handle, hwparams, exact_rate, 0);
	if ( e< 0) {
		fprintf(stderr, "Error setting playback rate.\n");
		return(-1);
	}
	if (rate != exact_rate)
		fprintf(stderr, "*The playback rate %d changed to %d Hz\n", rate, exact_rate);
#if DEBUG > 0		
	else
		fprintf(stderr, "Playback sampling rate is set to %d\n", exact_rate);
#endif

	/* Set number of channels */
	if ((e = snd_pcm_hw_params_set_channels(pcm_play_handle, hwparams, 2)) < 0) {
		fprintf(stderr, "*Error setting playback channels.\n");
		return(-1);
	}

/*
// This function call and the next have been replaced by the snd_pcm_hw_params_set_period_size_near() function call - N3SB December 2023
	// frame = bytes_per_sample x n_channel
	// period = frames transfered at a time (160 for voip, etc.)
	// we use two periods per buffer.
	if ((e = snd_pcm_hw_params_set_periods(pcm_play_handle, hwparams, n_periods_per_buffer, 0)) < 0) {
		fprintf(stderr, "*Error setting playback periods.\n");
		return(-1);
	}
*/

	// the buffer size is each periodsize x n_periods
	//	snd_pcm_uframes_t  n_frames= (buff_size  * n_periods_per_buffer)/8;
	//A larger buffer seems to hurt performance, reset to 'normal'
	//If a large pop occurs increase this by four (*4)
	snd_pcm_uframes_t  n_frames= (buff_size  * n_periods_per_buffer)/8;		
#if DEBUG > 0	
	printf("trying for buffer size of %ld\n", n_frames);
#endif
/*
// This function call and the previous have been replaced by the snd_pcm_hw_params_set_period_size_near() function call - N3SB December 2023
	e = snd_pcm_hw_params_set_buffer_size_near(pcm_play_handle, hwparams, &n_frames);
	if (e < 0) {
		    fprintf(stderr, "*Error setting playback buffersize.\n");
		    return(-1);
	}
*/
	// This function call replaces the two function calls above - N3SB December 2023
	e = snd_pcm_hw_params_set_period_size_near(pcm_play_handle, hwparams, &n_frames, 0);
	if (e < 0) {
		    fprintf(stderr, "*Error setting playback buffersize.\n");
		    return(-1);
	}

	if (snd_pcm_hw_params(pcm_play_handle, hwparams) < 0) {
		fprintf(stderr, "*Error setting playback HW params.\n");
		return(-1);
	}

	// get the current swparams
    e = snd_pcm_sw_params_current(pcm_play_handle, swparams);
    if (e < 0) {
        printf("Unable to determine current swparams for playback: %s\n", snd_strerror(e));
	}

    e = snd_pcm_sw_params_set_start_threshold(pcm_play_handle, swparams, (8192) );
    if (e < 0) {
        printf("Unable to set start threshold mode for playback: %s\n", snd_strerror(e));
    }


#if DEBUG > 0
	printf("PCM Playback Buffer Size: %d\n",snd_pcm_avail(pcm_play_handle));
	puts("All hw params set to play sound");
#endif	

	return 0;
}


int sound_start_loopback_capture(char *device){

	snd_pcm_hw_params_alloca(&hloop_params);
	
#if DEBUG > 0	
	printf ("opening audio loopback tx stream to %s\n", device); 
#endif
	int e = snd_pcm_open(&loopback_capture_handle, device, capture_stream, 0);
	
	if (e < 0) {
		fprintf(stderr, "Err: Opening loop capture  %s: %s\n", device, snd_strerror(e));
		return -1;
	}

	e = snd_pcm_hw_params_any(loopback_capture_handle, hloop_params);

	if (e < 0) {
		fprintf(stderr, "*Error setting capture access (%d)\n", e);
		return(-1);
	}

	e = snd_pcm_hw_params_set_access(loopback_capture_handle, hloop_params, SND_PCM_ACCESS_RW_INTERLEAVED);
	if (e < 0) {
		fprintf(stderr, "*Error setting capture access.\n");
		return(-1);
	}

  /* Set sample format */
	e = snd_pcm_hw_params_set_format(loopback_capture_handle, hloop_params, SND_PCM_FORMAT_S32_LE);
	if (e < 0) {
		fprintf(stderr, "*Error setting loopback capture format.\n");
		return(-1);
	}

	/* Set sample rate. If the exact rate is not supported */
	/* by the hardware, use nearest possible rate.         */ 
	exact_rate = 48000;
	//printf("Setting loopback capture rate to %d\n", exact_rate);
	e = snd_pcm_hw_params_set_rate(loopback_capture_handle, hloop_params, exact_rate, 0);
	if ( e< 0) {
		fprintf(stderr, "*Error setting loopback capture rate.\n");
		return(-1);
	}

	if (48000 != exact_rate)
		fprintf(stderr, "#The loopback capture rate set to %d Hz\n", exact_rate);

	/* Set number of channels */
	if ((e = snd_pcm_hw_params_set_channels(loopback_capture_handle, hloop_params, 2)) < 0) {
		fprintf(stderr, "*Error setting loopback capture channels.\n");
		return(-1);
	}

/*
	//printf("%d: set the #channels\n", __LINE__, 2);
	// Set number of periods. Periods used to be called fragments.
	if ((e = snd_pcm_hw_params_set_periods(loopback_capture_handle, hloop_params, n_periods_per_buffer, 0)) < 0) {
		fprintf(stderr, "*Error setting loopback capture periods.\n");
		return(-1);
	}

	// the buffer size is each periodsize x n_periods
	snd_pcm_uframes_t  n_frames= (buff_size  * n_periods_per_buffer) / 8;
	// printf("trying for buffer size of %ld\n", n_frames);
	e = snd_pcm_hw_params_set_buffer_size_near(loopback_capture_handle, hloop_params, &n_frames);
	if (e < 0) {
		    fprintf(stderr, "*Error setting loopback capture buffersize.\n");
		    return(-1);
	}
*/
	// This function call and the previous have been replaced by the snd_pcm_hw_params_set_period_size_near() function call - N3SB December 2023
	snd_pcm_uframes_t  n_frames= (buff_size  * n_periods_per_buffer) / 8;
	e = snd_pcm_hw_params_set_period_size_near(loopback_capture_handle, hloop_params, &n_frames, 0);
	if (e < 0) {
		    fprintf(stderr, "*Error setting loopback capture buffersize.\n");
		    return(-1);
	}
		    
	//printf("%d: set buffer to \n", __LINE__, n_frames);
	if (snd_pcm_hw_params(loopback_capture_handle, hloop_params) < 0) {
		fprintf(stderr, "*Error setting capture HW params.\n");
		return(-1);
	}

	//printf("%d: set  hwparams\n", __LINE__);
	/* set some parameters in the driver to handle the latencies */
	snd_pcm_sw_params_malloc(&sloop_params);
	if((e = snd_pcm_sw_params_current(loopback_capture_handle, sloop_params)) < 0){
		fprintf(stderr, "Error getting current loopback capture sw params : %s\n", snd_strerror(e));
		return (-1);
	}
	
	if ((e = snd_pcm_sw_params_set_start_threshold(loopback_capture_handle, sloop_params, 15)) < 0){
		fprintf(stderr, "Unable to set threshold mode for loopback capture\n");
	} 
	
	if ((e = snd_pcm_sw_params_set_stop_threshold(loopback_capture_handle, sloop_params, 1)) < 0){
		fprintf(stderr, "Unable to set stop threshold for loopback  capture\n");
	}
	
#if DEBUG > 0
	printf("Loopback Capture Buffer Size: %d\n",snd_pcm_avail(loopback_capture_handle));
	puts("All hw params set for loopback capture sound");
#endif	
	
	return 0;
}

/*
The capture is opened in a blocking mode, the read function will block until 
there are enough samples to return a block.
This ensures that the blocks are returned in perfect timing with the codec's clock
Once you process these captured samples and send them to the playback device, you
just wait for the next block to arrive 
*/

int sound_start_capture(char *device){
	snd_pcm_hw_params_alloca(&hwparams);
	
#if DEBUG > 0
	printf ("opening PCM Capture stream to %s\n", device); 
#endif
	int e = snd_pcm_open(&pcm_capture_handle, device,  	capture_stream, 0);
	
	if (e < 0) {
		fprintf(stderr, "Error opening PCM capture device %s: %s\n", device, snd_strerror(e));
		return -1;
	}

	e = snd_pcm_hw_params_any(pcm_capture_handle, hwparams);

	if (e < 0) {
		fprintf(stderr, "*Error setting PCM capture access (%d)\n", e);
		return(-1);
	}

	e = snd_pcm_hw_params_set_access(pcm_capture_handle, hwparams, SND_PCM_ACCESS_RW_INTERLEAVED);
	if (e < 0) {
		fprintf(stderr, "*Error setting PCM capture access.\n");
		return(-1);
	}

  /* Set sample format */
	e = snd_pcm_hw_params_set_format(pcm_capture_handle, hwparams, SND_PCM_FORMAT_S32_LE);
	if (e < 0) {
		fprintf(stderr, "*Error setting PCM capture format.\n");
		return(-1);
	}

	/* Set sample rate. If the exact rate is not supported */
	/* by the hardware, use nearest possible rate.         */ 
	exact_rate = rate;
	e = snd_pcm_hw_params_set_rate(pcm_capture_handle, hwparams, exact_rate, 0);
	if ( e< 0) {
		fprintf(stderr, "*Error setting PCM capture rate.\n");
		return(-1);
	}

	if (rate != exact_rate)
		fprintf(stderr, "#The PCM capture rate %d changed to %d Hz\n", rate, exact_rate);


	/* Set number of channels */
	if ((e = snd_pcm_hw_params_set_channels(pcm_capture_handle, hwparams, 2)) < 0) {
		fprintf(stderr, "*Error setting PCM capture channels.\n");
		return(-1);
	}

/*
	// Set number of periods. Periods used to be called fragments.
	if ((e = snd_pcm_hw_params_set_periods(pcm_capture_handle, hwparams, n_periods_per_buffer, 0)) < 0) {
		fprintf(stderr, "*Error setting capture periods.\n");
		return(-1);
	}

	// the buffer size is each periodsize x n_periods
	snd_pcm_uframes_t  n_frames= (buff_size  * n_periods_per_buffer)/ 8;
	//printf("trying for buffer size of %ld\n", n_frames);
	e = snd_pcm_hw_params_set_buffer_size_near(pcm_capture_handle, hwparams, &n_frames);
	if (e < 0) {
		    fprintf(stderr, "*Error setting PCM capture buffersize.\n");
		    return(-1);
	}
*/
	snd_pcm_uframes_t  n_frames= (buff_size  * n_periods_per_buffer)/ 8;
	// This function call replaces the two function calls above - N3SB December 2023
	e = snd_pcm_hw_params_set_period_size_near(pcm_capture_handle, hwparams, &n_frames, 0);
	if (e < 0) {
		    fprintf(stderr, "*Error setting PCM Capture buffersize.\n");
		    return(-1);
	}


	if (snd_pcm_hw_params(pcm_capture_handle, hwparams) < 0) {
		fprintf(stderr, "*Error setting PCM capture HW params.\n");
		return(-1);
	}

#if DEBUG > 0
	printf("Capture Buffer Size: %d\n",snd_pcm_avail(pcm_capture_handle));
	puts("All hw params set for PCM sound capture");
#endif	

	return 0;
}

// ---------------------------------------------------------------------------
// Scan ALSA PCM devices and find the first USB audio device.
//
// Fills out_play with the first plughw: playback device whose description
// contains "USB", and out_cap with the matching capture device on the same
// card.  Returns 1 if a USB device was found, 0 if none.
//
// The WM8731 audioinjector card is explicitly excluded -- we never want to
// "auto-detect" the RF codec as a USB headset.
// ---------------------------------------------------------------------------
int sound_find_usb_audio(char *out_play, char *out_cap, int maxlen)
{
    if (!out_play || !out_cap || maxlen < 1) return 0;
    out_play[0] = out_cap[0] = '\0';

    void **hints = NULL;
    if (snd_device_name_hint(-1, "pcm", &hints) < 0) return 0;

    int found_play = 0, found_cap = 0;

    for (void **h = hints; *h && (!found_play || !found_cap); h++) {
        char *name = snd_device_name_get_hint(*h, "NAME");
        char *desc = snd_device_name_get_hint(*h, "DESC");
        char *ioid = snd_device_name_get_hint(*h, "IOID"); // "Input","Output", or NULL=both

        if (name && desc && !strncmp(name, "plughw:", 7)) {
            // Check description contains "USB" (case-insensitive)
            int is_usb = 0;
            char desc_lower[256] = "";
            strncpy(desc_lower, desc, sizeof(desc_lower) - 1);
            for (char *p = desc_lower; *p; p++) *p = tolower((unsigned char)*p);
            if (strstr(desc_lower, "usb")) is_usb = 1;

            // Exclude the WM8731 / audioinjector RF codec
            if (strstr(name, "audioinjectorpi")) is_usb = 0;
            if (strstr(desc_lower, "audioinjector")) is_usb = 0;

            if (is_usb) {
                int is_output = (!ioid || strcmp(ioid, "Input")  != 0);
                int is_input  = (!ioid || strcmp(ioid, "Output") != 0);

                if (!found_play && is_output) {
                    strncpy(out_play, name, maxlen - 1);
                    out_play[maxlen - 1] = '\0';
                    found_play = 1;
                    fprintf(stderr, "USB audio out detected: %s (%s)\n", name, desc);
                }
                if (!found_cap && is_input) {
                    strncpy(out_cap, name, maxlen - 1);
                    out_cap[maxlen - 1] = '\0';
                    found_cap = 1;
                    fprintf(stderr, "USB audio in  detected: %s (%s)\n", name, desc);
                }
            }
        }
        if (name) free(name);
        if (desc) free(desc);
        if (ioid) free(ioid);
    }
    snd_device_name_free_hint(hints);
    return (found_play || found_cap) ? 1 : 0;
}

// ---------------------------------------------------------------------------
// Probe whether a named ALSA PCM playback device is actually available.
// Returns 1 if the device opens successfully, 0 if it cannot be opened
// (e.g. the USB dongle is not plugged in).
// Used by sbitx_gtk.c at startup to avoid assertion crashes from ALSA
// mixer calls against a card that has no card index.
// ---------------------------------------------------------------------------
int sound_usb_device_present(const char *device)
{
    if (!device || !device[0]) return 0;
    snd_pcm_t *handle = NULL;
    int err = snd_pcm_open(&handle, device, SND_PCM_STREAM_PLAYBACK,
                           SND_PCM_NONBLOCK);
    if (err < 0) {
        fprintf(stderr,
                "USB audio device '%s' not available at startup "
                "(err %d: %s) – falling back to default audio\n",
                device, err, snd_strerror(err));
        return 0;
    }
    snd_pcm_close(handle);
    return 1;
}

// ---------------------------------------------------------------------------
// Headset speaker output -- 48kHz stereo, non-blocking, best-effort.
// Returns 0 on success, -1 on failure (headset simply won't be used).
// ---------------------------------------------------------------------------
static int sound_start_usb_audio_play(const char *device)
{
    snd_pcm_hw_params_t *hp;
    snd_pcm_hw_params_alloca(&hp);

    int e = snd_pcm_open(&usb_audio_play_handle, device, SND_PCM_STREAM_PLAYBACK, 0);
    if (e < 0) {
        fprintf(stderr, "USB audio out: cannot open %s: %s\n", device, snd_strerror(e));
        usb_audio_play_handle = 0;
        return -1;
    }
    snd_pcm_hw_params_any(usb_audio_play_handle, hp);
    snd_pcm_hw_params_set_access(usb_audio_play_handle, hp, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(usb_audio_play_handle, hp, SND_PCM_FORMAT_S32_LE);
    unsigned int rate = 48000;
    snd_pcm_hw_params_set_rate_near(usb_audio_play_handle, hp, &rate, 0);
    snd_pcm_hw_params_set_channels(usb_audio_play_handle, hp, 2);
    snd_pcm_uframes_t frames = 1024;
    snd_pcm_hw_params_set_period_size_near(usb_audio_play_handle, hp, &frames, 0);
    if (snd_pcm_hw_params(usb_audio_play_handle, hp) < 0) {
        fprintf(stderr, "USB audio out: hw params failed for %s\n", device);
        snd_pcm_close(usb_audio_play_handle);
        usb_audio_play_handle = 0;
        return -1;
    }
    snd_pcm_prepare(usb_audio_play_handle);
    fprintf(stderr, "USB audio out opened: %s @ 48kHz\n", device);
    return 0;
}

// ---------------------------------------------------------------------------
// Headset mic capture -- 48kHz stereo, non-blocking.
// ---------------------------------------------------------------------------
static int sound_start_usb_audio_capture(const char *device)
{
    snd_pcm_hw_params_t *hp;
    snd_pcm_hw_params_alloca(&hp);

    // Open non-blocking so the main audio loop never stalls waiting for
    // headset mic frames -- if none are ready we simply keep the WM8731 audio.
    int e = snd_pcm_open(&usb_audio_cap_handle, device, SND_PCM_STREAM_CAPTURE,
                         SND_PCM_NONBLOCK);
    if (e < 0) {
        fprintf(stderr, "USB audio mic: cannot open %s: %s\n", device, snd_strerror(e));
        usb_audio_cap_handle = 0;
        return -1;
    }
    snd_pcm_hw_params_any(usb_audio_cap_handle, hp);
    snd_pcm_hw_params_set_access(usb_audio_cap_handle, hp, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(usb_audio_cap_handle, hp, SND_PCM_FORMAT_S32_LE);
    unsigned int rate = 48000;
    snd_pcm_hw_params_set_rate_near(usb_audio_cap_handle, hp, &rate, 0);
    snd_pcm_hw_params_set_channels(usb_audio_cap_handle, hp, 2);
    snd_pcm_uframes_t frames = 1024;
    snd_pcm_hw_params_set_period_size_near(usb_audio_cap_handle, hp, &frames, 0);
    if (snd_pcm_hw_params(usb_audio_cap_handle, hp) < 0) {
        fprintf(stderr, "USB audio mic: hw params failed for %s\n", device);
        snd_pcm_close(usb_audio_cap_handle);
        usb_audio_cap_handle = 0;
        return -1;
    }
    snd_pcm_prepare(usb_audio_cap_handle);
    fprintf(stderr, "USB audio mic opened: %s @ 48kHz\n", device);
    return 0;
}

// ---------------------------------------------------------------------------
// Set the playback volume on a USB audio device.
//
// USB audio devices expose their volume under different mixer element names
// depending on the chipset ("PCM", "Speaker", "Headphone", "Master").
// This function derives the hw: mixer card name from the plughw: PCM device
// name (e.g. "plughw:CARD=Headset,DEV=0" -> "hw:CARD=Headset"), then tries
// each common element name until one succeeds.  Failures are silent so the
// WM8731 volume path is never disrupted.
//
// volume_pct: 0-100
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// Internal helper: derive "hw:CARD=X" from "plughw:CARD=X,DEV=N"
// ---------------------------------------------------------------------------
static void usb_derive_hw_card(const char *plughw_device, char *hw_card, int hw_card_len)
{
    hw_card[0] = '\0';
    if (!plughw_device || !plughw_device[0]) return;
    const char *p = plughw_device;
    if      (!strncmp(p, "plughw:", 7)) p += 7;
    else if (!strncmp(p, "hw:", 3))    p += 3;
    char card_spec[48] = "";
    const char *comma = strchr(p, ',');
    if (comma)
        snprintf(card_spec, sizeof(card_spec), "%.*s", (int)(comma - p), p);
    else
        snprintf(card_spec, sizeof(card_spec), "%s", p);
    snprintf(hw_card, hw_card_len, "hw:%s", card_spec);
}

// ---------------------------------------------------------------------------
// Set the playback volume on a USB audio device.
//
// Enumerates all mixer elements and scores them by name to find the speaker
// output rather than the mic sidetone (Mic Playback Volume).
// Scoring for playback:
//   "Speaker"   in name  ->  3  (GeneralPlus "Speaker Playback Volume")
//   "Headphone" in name  ->  2
//   "PCM"       in name  ->  1
//   "Mic"       in name  -> -1  (reject: this is mic sidetone, not speaker)
//   anything else        ->  0
//
// volume_pct: 0-100
// ---------------------------------------------------------------------------
void sound_usb_set_volume(const char *plughw_device, int volume_pct)
{
    if (!plughw_device || !plughw_device[0]) return;

    char hw_card[64];
    usb_derive_hw_card(plughw_device, hw_card, sizeof(hw_card));

    snd_mixer_t *handle = NULL;
    if (snd_mixer_open(&handle, 0) < 0) return;
    if (snd_mixer_attach(handle, hw_card) < 0) {
        snd_mixer_close(handle);
        return;
    }
    snd_mixer_selem_register(handle, NULL, NULL);
    snd_mixer_load(handle);

    snd_mixer_elem_t *best = NULL;
    int best_score = -99;

    for (snd_mixer_elem_t *e = snd_mixer_first_elem(handle); e;
         e = snd_mixer_elem_next(e))
    {
        if (snd_mixer_elem_get_type(e) != SND_MIXER_ELEM_SIMPLE) continue;
        if (!snd_mixer_selem_has_playback_volume(e)) continue;
        long mn, mx;
        snd_mixer_selem_get_playback_volume_range(e, &mn, &mx);
        if (mn == mx) continue;

        snd_mixer_selem_id_t *sid;
        snd_mixer_selem_id_alloca(&sid);
        snd_mixer_selem_get_id(e, sid);
        const char *nm = snd_mixer_selem_id_get_name(sid);

        int score = 0;
        if (strstr(nm, "Mic") || strstr(nm, "mic"))          score = -1;
        else if (strstr(nm, "Speaker") || strstr(nm, "speaker")) score =  3;
        else if (strstr(nm, "Headphone"))                    score =  2;
        else if (strstr(nm, "PCM"))                          score =  1;

        if (score > best_score) { best_score = score; best = e; }
    }

    if (best) {
        snd_mixer_selem_id_t *sid;
        snd_mixer_selem_id_alloca(&sid);
        snd_mixer_selem_get_id(best, sid);
        long mn, mx;
        snd_mixer_selem_get_playback_volume_range(best, &mn, &mx);
        long val = mn + (long)(volume_pct * (mx - mn) / 100);
        snd_mixer_selem_set_playback_volume_all(best, val);
        static int logged = 0;
        if (!logged) {
            fprintf(stderr, "USB playback vol: %s[%s] range %ld..%ld\n",
                    hw_card, snd_mixer_selem_id_get_name(sid), mn, mx);
            logged = 1;
        }
    }
    snd_mixer_close(handle);
}

// ---------------------------------------------------------------------------
// Set the capture (mic) gain on a USB audio device.
//
// Scoring for capture:
//   "Mic"+"Capture" in name  ->  3  (GeneralPlus "Mic Capture Volume")
//   "Mic"           in name  ->  2
//   "Capture"       in name  ->  1
//   "Playback"      in name  -> -1  (reject: playback element)
//   anything else            ->  0
//
// gain_pct: 0-100
// ---------------------------------------------------------------------------
void sound_usb_set_capture(const char *plughw_device, int gain_pct)
{
    if (!plughw_device || !plughw_device[0]) return;

    char hw_card[64];
    usb_derive_hw_card(plughw_device, hw_card, sizeof(hw_card));

    snd_mixer_t *handle = NULL;
    if (snd_mixer_open(&handle, 0) < 0) return;
    if (snd_mixer_attach(handle, hw_card) < 0) {
        snd_mixer_close(handle);
        return;
    }
    snd_mixer_selem_register(handle, NULL, NULL);
    snd_mixer_load(handle);

    snd_mixer_elem_t *best = NULL;
    int best_score = -99;

    for (snd_mixer_elem_t *e = snd_mixer_first_elem(handle); e;
         e = snd_mixer_elem_next(e))
    {
        if (snd_mixer_elem_get_type(e) != SND_MIXER_ELEM_SIMPLE) continue;
        if (!snd_mixer_selem_has_capture_volume(e)) continue;
        long mn, mx;
        snd_mixer_selem_get_capture_volume_range(e, &mn, &mx);
        if (mn == mx) continue;

        snd_mixer_selem_id_t *sid;
        snd_mixer_selem_id_alloca(&sid);
        snd_mixer_selem_get_id(e, sid);
        const char *nm = snd_mixer_selem_id_get_name(sid);

        int has_mic     = (strstr(nm, "Mic")     || strstr(nm, "mic"))     ? 1 : 0;
        int has_cap     = (strstr(nm, "Capture") || strstr(nm, "capture")) ? 1 : 0;
        int has_play    = (strstr(nm, "Playback")|| strstr(nm, "playback"))? 1 : 0;

        int score = 0;
        if (has_play && !has_cap)       score = -1;
        else if (has_mic && has_cap)    score =  3;
        else if (has_mic)               score =  2;
        else if (has_cap)               score =  1;

        if (score > best_score) { best_score = score; best = e; }
    }

    if (best) {
        snd_mixer_selem_id_t *sid;
        snd_mixer_selem_id_alloca(&sid);
        snd_mixer_selem_get_id(best, sid);
        long mn, mx;
        snd_mixer_selem_get_capture_volume_range(best, &mn, &mx);
        long val = mn + (long)(gain_pct * (mx - mn) / 100);
        snd_mixer_selem_set_capture_volume_all(best, val);
        static int logged = 0;
        if (!logged) {
            fprintf(stderr, "USB capture gain: %s[%s] range %ld..%ld\n",
                    hw_card, snd_mixer_selem_id_get_name(sid), mn, mx);
            logged = 1;
        }
    }
    snd_mixer_close(handle);
}

// ---------------------------------------------------------------------------
// Enable or disable the USB mic capture switch.
// enable=1 turns the mic on; enable=0 mutes it at the hardware switch level.
// Uses the same element-enumeration approach as set/capture to find the
// capture switch regardless of what the firmware calls it.
// ---------------------------------------------------------------------------
void sound_usb_enable_capture(const char *plughw_device, int enable)
{
    if (!plughw_device || !plughw_device[0]) return;

    char hw_card[64];
    usb_derive_hw_card(plughw_device, hw_card, sizeof(hw_card));

    snd_mixer_t *handle = NULL;
    if (snd_mixer_open(&handle, 0) < 0) return;
    if (snd_mixer_attach(handle, hw_card) < 0) {
        snd_mixer_close(handle);
        return;
    }
    snd_mixer_selem_register(handle, NULL, NULL);
    snd_mixer_load(handle);

    /* Find and set every capture switch element on the card.
       This catches "Mic Capture Switch" and any other capture
       enable controls the device exposes. */
    for (snd_mixer_elem_t *e = snd_mixer_first_elem(handle); e;
         e = snd_mixer_elem_next(e))
    {
        if (snd_mixer_elem_get_type(e) != SND_MIXER_ELEM_SIMPLE) continue;
        if (!snd_mixer_selem_has_capture_switch(e)) continue;

        snd_mixer_selem_id_t *sid;
        snd_mixer_selem_id_alloca(&sid);
        snd_mixer_selem_get_id(e, sid);
        const char *nm = snd_mixer_selem_id_get_name(sid);

        /* Only touch mic/capture switches, not playback switches */
        int is_cap = (strstr(nm, "Capture") || strstr(nm, "capture") ||
                      strstr(nm, "Mic")     || strstr(nm, "mic"));
        if (!is_cap) continue;

        snd_mixer_selem_set_capture_switch_all(e, enable);
#if DEBUG > 0
        fprintf(stderr, "USB capture switch '%s' -> %s\n",
                nm, enable ? "on" : "off");
#endif
    }
    snd_mixer_close(handle);
}

int sound_start_loopback_play(char *device){
	//found out the correct device through aplay -L (for pcm devices)

	snd_pcm_hw_params_alloca(&hwparams);	//more alloc

#if DEBUG > 0
	printf ("opening Loopback Play stream to %s\n", device); 
#endif
	int e = snd_pcm_open(&loopback_play_handle, device, play_stream, 0);	// was SND_PCM_NONBLOCK
	
	if (e < 0) {
		fprintf(stderr, "Error opening loopback playback device %s: %s\n", device, snd_strerror(e));
		return -1;
	}

	e = snd_pcm_hw_params_any(loopback_play_handle, hwparams);

	if (e < 0) {
		fprintf(stderr, "*Error getting loopback playback params (%d)\n", e);
		return(-1);
	}

	e = snd_pcm_hw_params_set_access(loopback_play_handle, hwparams, SND_PCM_ACCESS_RW_INTERLEAVED);
	if (e < 0) {
		fprintf(stderr, "*Error setting loopback Play access.\n");
		return(-1);
	}

  /* Set sample format */
	e = snd_pcm_hw_params_set_format(loopback_play_handle, hwparams, SND_PCM_FORMAT_S32_LE);
	if (e < 0) {
		fprintf(stderr, "*Error setting loopback Play format.\n");
		return(-1);
	}

	/* Set sample rate. If the exact rate is not supported */
	/* by the hardware, use nearest possible rate.         */ 
	exact_rate = 48000;
	e = snd_pcm_hw_params_set_rate(loopback_play_handle, hwparams, exact_rate, 0);
	if ( e< 0) {
		fprintf(stderr, "Error setting playback rate.\n");
		return(-1);
	}
	if (48000 != exact_rate)
		fprintf(stderr, "*The loopback playback rate %d changed to %d Hz\n", rate, exact_rate);


	/* Set number of channels */
	if ((e = snd_pcm_hw_params_set_channels(loopback_play_handle, hwparams, 2)) < 0) {
		fprintf(stderr, "*Error setting playback channels.\n");
		return(-1);
	}

/*
	// frame = bytes_per_sample x n_channel
	// period = frames transfered at a time (160 for voip, etc.)
	// we use two periods per buffer.
	if ((e = snd_pcm_hw_params_set_periods(loopback_play_handle, hwparams, 8, 0)) < 0) {
		fprintf(stderr, "*Error setting playback periods.\n");
		return(-1);
	}

	// the buffer size is each periodsize x n_periods
	snd_pcm_uframes_t  n_frames= (buff_size  * n_periods_per_buffer)/8;
	//lets pump it up to see if we can reduce the dropped frames
	n_frames *= 4;
	//printf("trying for loopback buffer size of %ld\n", n_frames);
	e = snd_pcm_hw_params_set_buffer_size_near(loopback_play_handle, hwparams, &n_frames);
	if (e < 0) {
		    fprintf(stderr, "*Error setting loopback playback buffersize.\n");
		    return(-1);
	}
*/
	// This function call replaces the two function calls above - N3SB December 2023
	snd_pcm_uframes_t  n_frames= (buff_size  * n_periods_per_buffer)/8;
	//lets pump it up to see if we can reduce the dropped frames
	n_frames *= 4;	
	e = snd_pcm_hw_params_set_period_size_near(loopback_play_handle, hwparams, &n_frames, 0);
	if (e < 0) {
		    fprintf(stderr, "*Error setting loopback play buffersize.\n");
		    return(-1);
	}

	if (snd_pcm_hw_params(loopback_play_handle, hwparams) < 0) {
		fprintf(stderr, "*Error setting loopback playback HW params.\n");
		return(-1);
	}

#if DEBUG > 0
	printf("Loopback Playback Buffer Size: %d\n",snd_pcm_avail(pcm_capture_handle));
	puts("All hw params set for Loopback Playback");
#endif	

	return 0;
}

// this is only a test process to be substituted to try loopback 
// it was used to debug timing errors
void sound_process2(int32_t *input_i, int32_t *input_q, int32_t *output_i, int32_t *output_q, int n_samples){
 
	for (int i= 0; i < n_samples; i++){
		output_i[i] = input_q[i];
		output_q[i] = 0;
	}	
}

//check that we haven't free()-ed up the hwparams block
//don't call this function at all until that is fixed
//you don't have to call it anyway
void sound_stop(){
	snd_pcm_drop(pcm_play_handle);
	snd_pcm_drain(pcm_play_handle);

	snd_pcm_drop(pcm_capture_handle);
	snd_pcm_drain(pcm_capture_handle);
}
//Reset alsa looback playback device, fixes compounding latency issues - n1qm
//Approx .07 seconds will be lost every 5 minutes
void sound_reset(int force){
	static struct timespec gettime;
	clock_gettime(CLOCK_MONOTONIC, &gettime);
	
	long int ltv = gettime.tv_sec; // get time
	if (force !=1 && ltv - reset_loopback_interval < last_loopback_reset)
		return;
	
	snd_pcm_reset(loopback_play_handle);

	last_loopback_reset = ltv;
#if DEBUG > 0
	printf("%d: Resetting loopback_play_handle to deal with latency\n",ltv);
#endif
}

static int count = 0;
static struct timespec gettime_now;
//static long int last_time = 0;
static long int last_sec = 0;
static int nframes = 0;
int32_t	resample_in[10000];
int32_t	resample_out[10000];

struct timeval GetTimeStamp()
{
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return tv;
}
struct timeval tv;
signed long pcm_read_old_time = 0;
signed long pcm_read_new_time = 0;
signed long delta_time = 0;
unsigned long samples_read = 0;
unsigned long samples_written = 0;
static unsigned long loop_counter = 0;

int last_second = 0;
int nsamples = 0;
//int	played_samples = 0;

unsigned long sbitx_millis(){
	return sound_millis;
}

int sound_loop(){
	int32_t		*line_in, *line_out, *data_in, *data_out, 
						*input_i, *output_i, *input_q, *output_q;
  int pcmreturn, i, j, loopreturn;
  short s1, s2;
  int frames;
  int pcm_read_avail;
  int pcm_write_avail;

	//we allocate enough for two channels of int32_t sized samples	
  data_in = (int32_t *)malloc(buff_size * 2);
  line_in = (int32_t *)malloc(buff_size * 2);
  line_out = (int32_t *)malloc(buff_size * 2);
  data_out = (int32_t *)malloc(buff_size * 2);
  input_i = (int32_t *)malloc(buff_size * 2);
  output_i = (int32_t *)malloc(buff_size * 2);
  input_q = (int32_t *)malloc(buff_size * 2);
  output_q = (int32_t *)malloc(buff_size * 2);

  frames = buff_size / 8;


  if (pcm_play_handle)      snd_pcm_prepare(pcm_play_handle);
  if (loopback_play_handle) snd_pcm_prepare(loopback_play_handle);

/*  
  pcmreturn = snd_pcm_writei(pcm_play_handle, data_out, frames*2);		// Get a head start on filling the queue
#if DEBUG > 0  
  printf("Pre-filling play and loopback queues\n");
  printf("Playback buffer filled with %d samples\n",pcmreturn);
#endif
  pcmreturn = snd_pcm_writei(loopback_play_handle, data_out, frames);		// Get a head start on filling the queue
#if DEBUG > 0
  printf("Loopback buffer filled with %d samples\n",pcmreturn);  
#endif
*/
	//Note: the virtual cable samples queue should be flushed at the start of tx
 	qloop.stall = 1;

// ******************************************************************************************************** The Big Loop starts here

  while(sound_thread_continue) {

		//restart the pcm capture if there is an error reading the samples
		//this is opened as a blocking device, hence we derive accurate timing 

		//last_time = gettime_now.tv_nsec/1000;
		
		// printf("%d\n", last_time);
		
		
#if DEBUG > 1	
		pcm_read_avail = snd_pcm_avail(pcm_capture_handle);
		tv = GetTimeStamp(); // get time
		pcm_read_new_time= 1000000 * tv.tv_sec + tv.tv_usec; // Store time in microseconds
		delta_time = pcm_read_new_time - pcm_read_old_time;
		if ((delta_time > 11667) || (delta_time < 9667))	// Loop should iterate every 10667 microseconds
		{
			printf("Loop Counter: %d, Loop Time: %d, Samples available: %d\n", loop_counter, delta_time, pcm_read_avail);
		}
		pcm_read_old_time = pcm_read_new_time;
#endif
		
		// Safety: if the handle was closed during sound_restart(), exit the loop
		if (!pcm_capture_handle) break;

		while ((pcmreturn = snd_pcm_readi(pcm_capture_handle, data_in, frames)) < 0)
		{
			// Exit immediately if handle was closed or thread told to stop
			if (!sound_thread_continue || !pcm_capture_handle) goto sound_loop_exit;
			if (pcmreturn == -ENODEV || pcmreturn == -EBADF) goto sound_loop_exit;
			result = snd_pcm_prepare(pcm_capture_handle);
#if DEBUG > 0
			printf("**** PCM Capture Error: %s  count = %d\n",snd_strerror(pcmreturn), pcm_capture_error++);
#endif
		}
#if DEBUG > 1		
		printf("Delta Time: %d, Available output sample storage: %d\n", delta_time, snd_pcm_avail(pcm_play_handle));
#endif		
		samples_read += pcmreturn;
		
		i = 0; 
		j = 0;
#if DEBUG > 0
		if (pcmreturn < 1024)
			printf("\n----PCM Read Size = %d\n",pcmreturn);
#endif
		int ret_card = pcmreturn;

		if (use_virtual_cable)
		{
			//printf(" we have %d in qloop, writing now\n", q_length(&qloop));
			// if don't we have enough to last two iterations loop back...
			if (q_length(&qloop) < pcmreturn)
			{
#if DEBUG > -1
				puts(" skipping\n");
#endif
				continue;
			}
	
			//copy 1024 samples from the queue.
			//i = 0;
			//j = 0;

			
			for (int samples  = 0; samples < 1024; samples++)
			{
				input_i[samples] = input_q[samples] = q_read(&qloop);
				//j++; 
			}
			//played_samples += 1024;
		}  // end for use_virtual_cable test
		else 
		{
			while (i < ret_card)
			{
				input_i[i] = data_in[j++]/2;
				input_q[i] = data_in[j++]/2;
				i++;
			}
		}
  	clock_gettime(CLOCK_MONOTONIC, &gettime_now);
		sound_millis = (gettime_now.tv_sec * 1000) + (gettime_now.tv_nsec/1000000);
		// printf("\n-%d %ld %d\n", count++, nsamples, pcmreturn);

		// ---- Optional USB mic input ----
		// Only replace input_q[] with USB mic samples when NOT in virtual-cable
		// (loopback) mode.  In DIGI/PSK/RTTY modes use_virtual_cable=1 and
		// input_q[] is already sourced from the fldigi loopback queue -- the
		// USB mic must not overwrite it or those modes will stop working.
		if (usb_audio_cap_handle && !use_virtual_cable) {
			static int32_t hcap_buf[2048]; // stereo 48kHz frame buffer
			int hframes = ret_card / 2;    // half the SDR frame count
			if (hframes > 1024) hframes = 1024;
			int hr = snd_pcm_readi(usb_audio_cap_handle, hcap_buf, hframes);
			if (hr == -EAGAIN) {
				// No data ready yet (non-blocking) -- keep WM8731 audio in input_q
				hr = 0;
			} else if (hr < 0) {
				// Recover from xrun or suspend
				snd_pcm_recover(usb_audio_cap_handle, hr, 1);
				hr = 0;
			}
			if (hr > 0) {
				// Upsample 48kHz -> 96kHz: each 48kHz sample s becomes two
				// consecutive 96kHz samples in input_q[] (flat mono array).
				// hcap_buf is stereo interleaved: [L0,R0, L1,R1, ...]
				// We use left channel only.
				for (int s = 0; s < hr; s++) {
					int32_t v = hcap_buf[s * 2]; // left channel of stereo pair
					input_q[s * 2]     = v;      // 96kHz sample n
					input_q[s * 2 + 1] = v;      // 96kHz sample n+1 (held)
				}
			}
		}

		sound_process(input_i, input_q, output_i, output_q, ret_card);

		i = 0; 
		j = 0;	
		//while (i < ret_card){
		//	data_out[j++] = output_i[i];
		//	data_out[j++] = output_q[i++];
		//}
		for (int p = 0; p < ret_card; p++) {
			//Process array for HW sound out
			data_out[p*2] = output_i[p];
			data_out[p*2+1] = output_q[p];
			//Process array for alsa loop back
			//if (p < ret_card/2)
			//	line_out[p*2] =line_out[p*2+1] = output_i[p*2];
		}
	if (!pcm_play_handle) goto sound_loop_exit;
	int framesize = ret_card;
	int offset = 0;
	int play_write_errors = 0;
	int pswitch = 0;
		
	while(framesize > 0)
	{
		do
		{
			pcmreturn = snd_pcm_avail(pcm_play_handle);
		} while ((pcmreturn == 0) || (pcmreturn == -11));
		
		do
		{
			pcmreturn = snd_pcm_writei(pcm_play_handle, data_out + offset, framesize);
		} while (pcmreturn == -11);
#if DEBUG > 0		
		if ((pcmreturn > 0) && (pcmreturn < 1024))
		{
			printf("#### Partial Write ####");	// would signify that the playback channel didn't accept all the samples
		}
#endif
		// if((pcmreturn < 0) && (pcmreturn != -11))	// also ignore "temporarily unavailable" errors
		if(pcmreturn < 0)
		{
			// Handle an error condition from the snd_pcm_writei function
#if DEBUG > 0			
			printf("Loop Counter: %d, Play PCM Write Error %d: %s  count = %d\n",loop_counter, pcmreturn, snd_strerror(pcmreturn), pcm_play_write_error++);
#endif
			if (pcmreturn == -EPIPE)
			{
#if DEBUG > 0
				printf("Samples Read: %d, Samples Written: %d, delta: %d, available %d\n", samples_read, samples_written, samples_read - samples_written, pcm_write_avail);
				printf("Available write buffer: %d\n", pcm_write_avail);
#endif
				snd_pcm_recover(pcm_play_handle, pcmreturn, 0);
				//If buffer underruns, let's also reset the playbook loop
				sound_reset(1);
			}

			
#if DEBUG > 0
				printf("Now Available write buffer: %d\n", snd_pcm_avail(pcm_play_handle));				
#endif			
			
		}
		if(pcmreturn >= 0)
		{
			// Calculate remaining number of samples to be sent and new position in sample array.
			// If all the samples were processed by the snd_pcm_writei function then framesize will be
			// zero and the while() loop will end.
#if DEBUG > 0
			if (pswitch == 1)
			{
				printf("%d ",pcmreturn);
			}				
#endif
			framesize -= pcmreturn;
			if ((framesize > 0)	&& (pswitch == 0))		
			{
#if DEBUG > 0
				printf("%d ",pcmreturn);
#endif
				pswitch = 1;				
			}
			offset += (pcmreturn * 2);
			samples_written += pcmreturn;
		}
		if (framesize == 0 && pswitch == 1) {
#if DEBUG > 0				
			printf("\n");
#endif				
			pswitch = 0;
			
		}
	}
	// End of new pcm play write routine

		// ---- Optional USB headset speaker output ----
		// The headset has no TX/RX relay, so it uses its own short 3-block
		// mute (32ms) on TX->RX rather than the full MUTE_MAX relay-settling
		// mute that sbitx.c applies to output_i[].  This means the headset
		// returns to RX audio faster than the WM8731 speaker.
		if (usb_audio_play_handle) {
			static int32_t hplay_buf[2048];
			static int usb_play_mute = 0;
			static int usb_prev_tx = 0;
			int usb_cur_tx = is_in_tx();
			int hframes = ret_card / 2;
			if (hframes > 1024) hframes = 1024;

			/* Arm short mute on TX->RX edge */
			if (usb_prev_tx && !usb_cur_tx)
				usb_play_mute = 3;
			usb_prev_tx = usb_cur_tx;

			if (!usb_cur_tx && !use_virtual_cable && usb_play_mute == 0) {
				/* Normal RX -- send demodulated audio */
				for (int s = 0; s < hframes; s++) {
					int32_t v = output_i[s * 2];
					hplay_buf[s * 2]     = v;
					hplay_buf[s * 2 + 1] = v;
				}
			} else {
				/* TX, DIGI, or post-TX mute -- silence */
				memset(hplay_buf, 0, hframes * 2 * sizeof(int32_t));
				if (!usb_cur_tx && usb_play_mute > 0)
					usb_play_mute--;
			}

			int hw = snd_pcm_writei(usb_audio_play_handle, hplay_buf, hframes);
			if (hw < 0 && hw != -EAGAIN) {
				snd_pcm_recover(usb_audio_play_handle, hw, 1);
				snd_pcm_writei(usb_audio_play_handle, hplay_buf, hframes);
			}
		}

#if DISABLE_LOOPBACK == 0

	if (loopback_play_handle) { // guard: handle may be NULL if loopback device absent
	//decimate the line out to half, ie from 96000 to 48000
	//play the received data (from left channel) to both of line out

	i = 0;
	j = 0;
	
	while (i < ret_card){
		//line_out[jj++] = output_i[ii] / LOOPBACK_LEVEL_DIVISOR;  // Left Channel. Reduce audio level to FLDIGI a bit
		//line_out[jj++] = output_i[ii] / LOOPBACK_LEVEL_DIVISOR;  // Right Channel. Note: FLDIGI does not use the this channel.
		line_out[j++] = line_out[j++] = output_i[i];	
		// The right channel can be used to output other integer values such as AGC, for capture by an
		// application such as audacity.
		i += 2;	// Skip a pair of samples to account for the 96K sample to 48K sample rate change.
	}


	// This is the new pcm loopback write routine
	framesize = (ret_card + 1) /2;		// only writing half the number of samples because of the slower channel rate
	offset = 0;

	while(framesize > 0)
	{
		do
		{
			pcmreturn = snd_pcm_avail(loopback_play_handle);
		} while ((pcmreturn == 0) || (pcmreturn == -11));
				
	//	printf("Writing %d frame to loopback\n", framesize);
	
		do
		{	
			pcmreturn = snd_pcm_writei(loopback_play_handle, line_out + offset, framesize);
		} while (pcmreturn == -11);
		
		// if((pcmreturn < 0) && (pcmreturn != -11))	// also ignore "temporarily unavailable" errors
		if(pcmreturn < 0)
		{  	// Handle an error condition from the snd_pcm_writei function
#if DEBUG > 0			
			printf("Loopback PCM Write %d bytes Error %d: %s  count = %d\n", framesize, pcmreturn, snd_strerror(pcmreturn), pcm_loopback_write_error++);
#endif

#if DEBUG <2
			snd_pcm_recover(loopback_play_handle, pcmreturn, 1);		// Does not provide detailed error message
			//printf("EPIPE in loop pcm play\n");
#else
			snd_pcm_recover(loopback_play_handle, pcmreturn, 0);		// Provides detailed error message
#endif				
		}
		
		if(pcmreturn >= 0)
		{
			// Calculate remaining number of samples to be sent and new position in sample array.
			// If all the samples were processed by the snd_pcm_writei function then framesize will be
			// zero and the while() loop will end.	
			framesize -= pcmreturn;
			offset += (pcmreturn * 2);
#if DEBUG > 0
			if (framesize > 0)
			{
				printf("\nLoopback - pcmreturn = %d\n", pcmreturn);
			}
#endif
		}
	}
	// End of new pcm loopback write routine
	} // end loopback_play_handle guard
    
#endif
    
		//played_samples += pcmreturn;
#if DEBUG > 0
	loop_counter++;		
#endif
  } // End of while (sound_thread_continue) loop
sound_loop_exit:
	//fclose(pf);
  printf("********Ending sound thread\n");
}


int loopback_loop(){
	int32_t		*line_in, *line_out, *data_in, *data_out, 
						*input_i, *output_i, *input_q, *output_q;
  int pcmreturn, i, j, loopreturn;
  short s1, s2;
  int frames;

	//we allocate enough for two channels of int32_t sized samples	
  data_in = (int32_t *)malloc(buff_size * 2);
  frames = buff_size / 8;
  if (loopback_capture_handle) snd_pcm_prepare(loopback_capture_handle);
	i = 0; 
	j = 0;
  while(sound_thread_continue) {

		//restart the loopback capture if there is an error reading the samples
		//this is opened as a blocking device, hence we derive accurate timing 

		//last_time = gettime_now.tv_nsec/1000;

		// Safety: if the handle was closed (e.g. during sound_restart), exit cleanly
		if (!loopback_capture_handle) goto loopback_exit;

		while ((pcmreturn = snd_pcm_readi(loopback_capture_handle, data_in, frames/2)) < 0){
			// If the thread has been told to stop (e.g. during sound_restart),
			// exit immediately rather than spinning on a closed/dead handle.
			if (!sound_thread_continue) goto loopback_exit;
			if (pcmreturn == -ENODEV || pcmreturn == -EBADF) goto loopback_exit;
			if (loopback_capture_handle) snd_pcm_prepare(loopback_capture_handle);
			//putchar('=');
		}
		if (!sound_thread_continue) break;
		// int ret_card = pcmreturn;

		//fill up a local buffer, take only the left channel	
		// i = 0; 
			
		for (i = 0; i < pcmreturn; i++){
			q_write(&qloop, data_in[j]);
			q_write(&qloop, data_in[j]);
			j += 2;
		}
		//nsamples += j;
		j=0;
		clock_gettime(CLOCK_MONOTONIC, &gettime_now);
		if (gettime_now.tv_sec != last_sec){
			last_sec = gettime_now.tv_sec;
			//printf("Next reset in %d\n", (long)reset_loopback_interval - (last_sec - last_loopback_reset));
			if (last_loopback_reset + (long)reset_loopback_interval < gettime_now.tv_sec)
				sound_reset(0);
			//if(use_virtual_cable)
//			printf("######sampling rate %d/%d\n", played_samples, nsamples);
			
			//nsamples = 0;
			//played_samples = 0;
			//count = 0;
			
		}

  }
loopback_exit:
  printf("********Ending loopback thread\n");
}

/*
We process the sound in a background thread.
It will call the user-supplied function sound_process()  
*/
void *sound_thread_function(void *ptr){
	char *device = (char *)ptr;
	struct sched_param sch;

	//switch to maximum priority
	sch.sched_priority = sched_get_priority_max(SCHED_FIFO);
	pthread_setschedparam(sound_thread, SCHED_FIFO, &sch);

// Open the PCM Capture Device
	int i = 0;
	for (i = 0; i < 10; i++){	
		if (sound_start_capture(device) == 0)
			break;
		fprintf(stderr, "*Error opening PCM Capture device");
		delay(1000); //wait for the sound system to bootup
		printf("Retrying the sound system %d\n", i);
	}
	if (i == 10){
	  fprintf(stderr, "*Error opening PCM Capture device - Aborting");
		return NULL;
	}

// Open the PCM Play Device	
	for (i = 0; i < 10; i++){
		if (sound_start_play(device) == 0)
			break;
		fprintf(stderr, "*Error opening PCM Play device");
		delay(1000); //wait for the sound system to bootup
		printf("Retrying the sound system %d\n", i);
	}
	if (i == 10){
	 fprintf(stderr, "*Error opening PCM Play device - Aborting");
		return NULL;
	}

	//Preset the last time the loopback was reset (now during init)
	clock_gettime(CLOCK_MONOTONIC_RAW, &gettime_now);
	last_loopback_reset = gettime_now.tv_sec;

// Open the Loopback Play Device
//  printf("opening loopback on plughw:CARD=Loopback,DEV=0 sound card\n");

	for (i = 0; i < 10; i++){
		if(sound_start_loopback_play(loopback_play_device) == 0)
			break;
		fprintf(stderr, "*Error opening Loopback Play device");
		delay(1000);
		printf("Retrying the sound system %d\n", i);
	}
	if (i == 10){
	 fprintf(stderr, "*Error opening Loopback Play device - Aborting");
		return NULL;
	}
	
	// Open optional headset devices if configured
	if (usb_audio_play_device[0])
		sound_start_usb_audio_play(usb_audio_play_device);
	if (usb_audio_cap_device[0])
		sound_start_usb_audio_capture(usb_audio_cap_device);

	sound_thread_continue = 1;
	sound_loop();
	sound_stop();
}

void *loopback_thread_function(void *ptr){
	struct sched_param sch;

	//switch to maximum priority
	sch.sched_priority = sched_get_priority_max(SCHED_FIFO);
	pthread_setschedparam(loopback_thread, SCHED_FIFO, &sch);
//	printf("loopback thread is %x\n", loopback_thread);
//  printf("opening loopback on plughw:CARD=Loopback_1,DEV=1 sound card\n");	

	int i = 0;
	for (i = 0; i < 10; i++){
		if (sound_start_loopback_capture(loopback_capture_device) == 0)
			break;
		fprintf(stderr, "*Error opening Loopback Capture device");
		delay(1000);
		printf("Retrying the sound system %d\n", i);
	}
	if (i == 10){
	 fprintf(stderr, "*Error opening Loopback Capture device - Aborting");
		return NULL;
	}
		
	
	sound_thread_continue = 1;
	loopback_loop();
	sound_stop();
}

int sound_thread_start(char *device){
	q_init(&qloop, 10240);
 	qloop.stall = 1;

	pthread_create( &sound_thread, NULL, sound_thread_function, (void*)device);
	sleep(1);
	pthread_create( &loopback_thread, NULL, loopback_thread_function, (void*)device);
}

void sound_thread_stop(){
	sound_thread_continue = 0;
}

void sound_input(int loop){
  if (loop){
    use_virtual_cable = 1;
	}
  else{
    use_virtual_cable = 0;
	}
}


//demo, uncomment it to test it out
/*
void sound_process(int32_t *input_i, int32_t *input_q, int32_t *output_i, int32_t *output_q, int n_samples){
 
	for (int i= 0; i < n_samples; i++){
		output_i[i] = input_i[i];
		output_q[i] = input_q[i];
	}	
}

void main(int argc, char **argv){
	sound_thread_start("plughw:CARD=audioinjectorpi,DEV=0");
	sleep(10);
	sound_thread_stop();
	sleep(10);
}
*/

