# RNNoise Integration — Scoping Notes

Status: **proposal only, no implementation yet.** For review before deciding
whether to proceed.

## What it is
[RNNoise](https://github.com/xiph/rnnoise) is a small recurrent neural
network (GRU-based) trained for real-time speech denoising. It ships as a
self-contained C library — no GPU, no Python runtime, no heavyweight ML
framework. It processes audio in fixed 480-sample (10ms @ 48kHz) frames and
outputs a per-frame voice-activity probability alongside the denoised audio.

## Why it's likely better than the current NR stack
The existing spectral subtraction and Wiener ANR stages both work from a
single noise estimate (`noise_est[]`) that's essentially "whatever was quiet
recently." RNNoise instead uses a model trained on a large corpus of speech +
noise pairs, so it distinguishes voice from non-voice content structurally,
not just by recent power level. It generally holds up better on bursty or
non-stationary noise (impulse noise, wind, keyboard clicks) where our current
per-bin power tracking lags.

## Integration points in the current pipeline

1. **Sample rate mismatch.** sBitx's DSP runs at 96kHz (`sampling_rate` in
   `sbitx.c`); RNNoise expects 48kHz mono. Needs a resample step in both
   directions around the RNNoise call, or running RNNoise on a decimated
   copy of the audio used only for the noise mask, then applying that mask
   back at 96kHz. The second approach avoids doing full up/downsampling of
   the audio path itself. Needs benchmarking to see which is cheaper.

2. **Where in the chain to insert it.** Best candidate is where `dsp_enabled`
   and `anr_enabled` currently branch (`sbitx.c` ~line 1502-1557) — RNNoise
   would likely become a third selectable NR mode there, not a fourth stage
   stacked on top of the existing two. Recommend making it mutually
   exclusive with the other two (radio button, not three checkboxes) since
   stacking independent NR algorithms is already a known issue (see the
   threshold-fix branch notes).

3. **Buffering.** RNNoise wants fixed 480-sample frames; sBitx's FFT block
   size is `MAX_BINS` (need to confirm exact value against 96kHz timing).
   Likely needs a small ring buffer to reframe audio into 10ms chunks
   independent of the existing FFT block size.

4. **Build system.** RNNoise is a small, dependency-free C library — should
   vendor cleanly as a git submodule (similar to how `ft8_lib` is already
   vendored) or a bundled source drop. No new system package dependencies
   expected.

5. **CPU budget.** RNNoise was designed to run on hardware far weaker than a
   Pi 4/5 (targets embedded VoIP endpoints). Real-time performance shouldn't
   be a concern, but needs to be confirmed empirically once running
   alongside the existing FFT/AGC/waterfall load on the same core(s).

## Open questions before implementation starts
- Do we want RNNoise as a strict replacement for spectral subtraction / ANR,
  or an additional selectable mode alongside them?
- Full audio-path resampling vs. mask-only approach — needs a quick
  prototype to compare CPU cost and audible quality.
- Should this be gated as an experimental/opt-in feature initially, given
  it changes the RX audio path more invasively than the threshold fix?

## Suggested next step
A small standalone prototype (off the main audio path, e.g. processing a
recorded WAV of a noisy band) to confirm audible improvement and rough CPU
cost, before touching `sbitx.c`'s live RX pipeline. This avoids risking
regressions in the actual receive audio while validating the idea.
