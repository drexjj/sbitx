/* RNNoise-based noise reduction for the sbitx RX speaker path.
 *
 * Pipeline (per rx_linear block of MAX_BINS/2 = 1024 samples @ 96 kHz):
 *
 *   96 kHz int32 speaker samples
 *     -> scale down to int16-float range     (/ RNN_SCALE)
 *     -> decimate 2:1 to 48 kHz              (audio is already band-limited
 *                                             well below 24 kHz by the RX
 *                                             bandpass FIR, so plain 2:1
 *                                             decimation adds no audible
 *                                             aliasing)
 *     -> ring buffer -> 480-sample frames -> rnnoise_process_frame()
 *     -> wet/dry mix with an identically delayed copy of the input
 *        (rnn_strength, 0-100). Full-strength neural gating swings band
 *        gains between ~0 and 1 as speech confidence changes, which is
 *        audible as volume pumping/breathing, especially with QSB. Mixing
 *        a fraction of the dry signal back in bounds the depth of those
 *        swings: at strength S the output can never drop below (1 - S/100)
 *        of the input, which masks pumping while retaining most of the
 *        noise reduction.
 *     -> interpolate 1:2 back to 96 kHz      (linear; images land > 43 kHz,
 *                                             far above the audio band)
 *     -> scale back up                        (* RNN_SCALE)
 *
 * Latency: one RNNoise frame (480 samples @ 48 kHz = 10 ms) plus partial
 * frame buffering, ~10-20 ms total. Until the first frame is filled the
 * output is briefly zero-padded, which is inaudible at enable time.
 *
 * Not thread-safe; called only from the sound-processing thread the same
 * way the existing DSP/ANR stages are.
 */

#include <string.h>
#include <math.h>
#include "sbitx_rnn.h"
#include "../rnnoise/include/rnnoise.h"

#define RNN_FRAME 480 /* rnnoise_get_frame_size() for the bundled model */

/* Speaker samples reach roughly +/-4e8 after demod scaling (see the
 * limiter threshold of 0.8 * 5e8 in rx_linear). Dividing by 16384 maps
 * that to roughly +/-24k, comfortably inside the int16-style range the
 * RNNoise model was trained on. */
#define RNN_SCALE 16384.0f

/* Ring buffers sized for several rx_linear blocks of headroom. */
#define RNN_RING (RNN_FRAME * 8)

/* Noise-reduction strength, 0-100 (percent wet). 100 = pure RNNoise
 * output (deepest noise reduction, most pumping); lower values blend the
 * dry signal back in. 70-85 is a good compromise on SSB. Set from the UI
 * via the RNNS field / \rnns console command. */
int rnn_strength = 80;

static DenoiseState *rnn_st = NULL;

static float in_ring[RNN_RING];
static int in_count = 0; /* valid 48 kHz samples waiting to be denoised */

/* wet = denoised, dry = identically delayed original, kept in lockstep */
static float wet_ring[RNN_RING];
static float dry_ring[RNN_RING];
static float vad_ring[RNN_RING]; /* smoothed VAD aligned with wet/dry */
static int out_head = 0; /* read position (shared by wet and dry) */
static int out_count = 0; /* valid denoised 48 kHz samples available */

/* VAD-adaptive mixing state and tuning.
 *
 * rnnoise_process_frame() returns the network's per-frame voice-activity
 * probability. On weak/fading signals that confidence sits in the
 * uncertain middle and the band gains chatter, which eats fragile speech.
 * Mitigation: as (smoothed) VAD rises, relax the wet mix so speech - even
 * weak speech - is protected from the gate, while frames the network is
 * confident are pure noise still get full suppression.
 *
 * effective_wet = wet_mix * (1 - (rnn_relax/100) * vad_smooth)
 *
 * rnn_relax (0-50, percent) is the relax depth: how far suppression backs
 * off at full speech confidence. Runtime-tunable from the UI (AINRV field
 * beside AINRS) for on-air tuning; 25 is the current default (v1's 50 was
 * reported as too much noise halo on weak signals).
 * VAD_ATTACK/RELEASE are per-10ms-frame smoothing coefficients: attack
 * reaches ~90% in ~3 frames (30 ms, catches syllable onsets); release
 * decays with a ~150 ms time constant so suppression recovers inside an
 * inter-word gap instead of persisting through it. */
#define VAD_ATTACK 0.55f
#define VAD_RELEASE 0.065f
int rnn_relax = 25;
static float vad_smooth = 0.0f;

static float last_out = 0.0f; /* previous 48 kHz sample, for interpolation */

void rnn_reset(void)
{
	in_count = 0;
	out_head = 0;
	out_count = 0;
	last_out = 0.0f;
	vad_smooth = 0.0f;
	if (rnn_st) {
		rnnoise_destroy(rnn_st);
		rnn_st = NULL;
	}
}

void rnn_process_speaker(int32_t *samples, int n_samples)
{
	int i;

	if (!rnn_st) {
		rnn_st = rnnoise_create(NULL); /* NULL = built-in model */
		if (!rnn_st)
			return; /* allocation failed: pass audio through untouched */
	}

	/* Decimate 96k -> 48k straight into the input ring. */
	for (i = 0; i + 1 < n_samples && in_count < RNN_RING; i += 2)
		in_ring[in_count++] = (float)samples[i] / RNN_SCALE;

	/* Denoise every complete 480-sample frame we have. */
	while (in_count >= RNN_FRAME && out_count <= RNN_RING - RNN_FRAME) {
		float frame[RNN_FRAME];
		memcpy(frame, in_ring, sizeof(frame));
		memmove(in_ring, in_ring + RNN_FRAME,
		        (in_count - RNN_FRAME) * sizeof(float));
		in_count -= RNN_FRAME;

		/* Keep the output rings compact at index 0. */
		if (out_head) {
			memmove(wet_ring, wet_ring + out_head,
			        out_count * sizeof(float));
			memmove(dry_ring, dry_ring + out_head,
			        out_count * sizeof(float));
			memmove(vad_ring, vad_ring + out_head,
			        out_count * sizeof(float));
			out_head = 0;
		}

		/* Dry copy first (rnnoise processes in place). The return value
		 * is the network's voice-activity probability for this frame,
		 * used below for VAD-adaptive mixing. */
		memcpy(dry_ring + out_count, frame, sizeof(frame));
		float vad = rnnoise_process_frame(rnn_st, frame, frame);
		memcpy(wet_ring + out_count, frame, sizeof(frame));

		/* Smooth the VAD with fast attack / slow release: speech onsets
		 * raise it within a frame or two, but it decays slowly so it
		 * doesn't flutter between words and syllables. */
		if (vad > vad_smooth)
			vad_smooth += VAD_ATTACK * (vad - vad_smooth);
		else
			vad_smooth += VAD_RELEASE * (vad - vad_smooth);

		/* Store the smoothed VAD for each sample of this frame so the
		 * mixing loop below stays aligned even across block boundaries. */
		for (int k = 0; k < RNN_FRAME; k++)
			vad_ring[out_count + k] = vad_smooth;

		out_count += RNN_FRAME;
	}

	/* Clamp strength and precompute the base mix once per block. */
	int s = rnn_strength;
	if (s < 0)
		s = 0;
	if (s > 100)
		s = 100;
	float base_wet = (float)s / 100.0f;

	int rv = rnn_relax;
	if (rv < 0)
		rv = 0;
	if (rv > 50)
		rv = 50;
	float relax = (float)rv / 100.0f;

	/* Interpolate 48k -> 96k back into the caller's block. Each output
	 * pair is (midpoint, sample) so the reconstructed stream stays
	 * aligned with the decimated one. The wet mix is relaxed per sample
	 * by the smoothed VAD (see above) so weak speech is protected while
	 * confident noise still gets the full configured suppression. If the
	 * output ring hasn't filled yet (first ~10 ms after enabling), emit
	 * silence for the shortfall rather than stale input. */
	int need = n_samples / 2;
	for (i = 0; i < need; i++) {
		float cur;
		if (out_count > 0) {
			float wet_mix =
			    base_wet * (1.0f - relax * vad_ring[out_head]);
			cur = wet_mix * wet_ring[out_head] +
			      (1.0f - wet_mix) * dry_ring[out_head];
			out_head++;
			out_count--;
		} else {
			cur = 0.0f;
		}
		samples[2 * i] = (int32_t)(0.5f * (last_out + cur) * RNN_SCALE);
		samples[2 * i + 1] = (int32_t)(cur * RNN_SCALE);
		last_out = cur;
	}
}
