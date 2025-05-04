class AudioPlaybackProcessor extends AudioWorkletProcessor {
    constructor(options) {
        super();
        
        const processorOptions = options.processorOptions || {};
        
        this.bufferSize = 4096;
        this.buffer = new Float32Array(this.bufferSize);
        this.writePos = 0;
        this.readPos = 0;
        this.availableSamples = 0;
        this.isReady = false;
        this.minBufferThreshold = 1536;
        
        this.inputSampleRate = processorOptions.inputSampleRate || 16000;
        this.outputSampleRate = processorOptions.outputSampleRate || sampleRate;
        this.sampleRateRatio = this.outputSampleRate / this.inputSampleRate;
        
        this.debug = processorOptions.debug || false;
        this.forceSilence = 0;
        this.sampleRateLogged = false;

        // Set default volume to 5 (0 dB gain)
        this.setVolume(11);

        this.port.onmessage = (event) => {
            if (event.data && typeof event.data === 'object' && event.data.command) {
                if (event.data.command === 'set_volume') {
                    const volume = Math.max(0, Math.min(10, event.data.value));
                    this.setVolume(volume);
                    this.port.postMessage({ status: "volume_set", value: volume });
                    return;
                }
                if (event.data.command === 'clear_buffer') {
                    this.buffer = new Float32Array(this.bufferSize);
                    this.writePos = 0;
                    this.readPos = 0;
                    this.availableSamples = 0;
                    this.isReady = false;
                    this.forceSilence = 30;
                    this.port.postMessage({ status: "buffer_cleared" });
                    if (this.debug) {
                        console.log('[RX Worklet] Buffer completely cleared and reset');
                    }
                    return;
                }
                return;
            }
            
            if (this.availableSamples >= this.bufferSize) {
                this.port.postMessage({ status: "buffer_full" });
                return;
            }
            
            if (!this.sampleRateLogged && this.debug) {
                console.log(`[RX Worklet] Audio context sample rate: ${sampleRate}Hz, Input rate: ${this.inputSampleRate}Hz`);
                this.sampleRateLogged = true;
            }

            const int16Data = new Int16Array(event.data);
            const float32Data = new Float32Array(int16Data.length);

            let min = 32767, max = -32768, sum = 0;
            for (let i = 0; i < int16Data.length; i++) {
                if (int16Data[i] < min) min = int16Data[i];
                if (int16Data[i] > max) max = int16Data[i];
                sum += int16Data[i];
            }
            const mean = sum / int16Data.length;
            
            for (let i = 0; i < int16Data.length; i++) {
                let sample = int16Data[i] / 32768.0;
                if (Math.abs(mean) > 100) {
                    sample = sample - (mean / 32768.0 * 0.5);
                }
                float32Data[i] = Math.max(-1.0, Math.min(1.0, sample));
            }

            let writeIndex = this.writePos;
            for (let i = 0; i < float32Data.length && this.availableSamples < this.bufferSize; i++) {
                this.buffer[writeIndex] = float32Data[i];
                writeIndex = (writeIndex + 1) % this.bufferSize;
                this.availableSamples++;
            }
            this.writePos = writeIndex;

            if (!this.isReady && this.availableSamples >= this.minBufferThreshold) {
                this.isReady = true;
                console.log("Buffer ready, starting playback");
            }

            this.port.postMessage({ status: "buffer_level", level: this.availableSamples / this.bufferSize });
        };
    }

    setVolume(volume) {
        // Map 0-10 to 0.0-4.0 gain
        // 0 = 0.0 (-100%), 5 = 1.0 (0%), 10 = 4.0 (+400%)
        const normalizedVolume = volume / 2.5; // Maps 0-10 to 0.0-4.0
        this.gain = normalizedVolume;
        if (this.debug) {
            console.log(`[RX Worklet] Volume set to ${volume}/10 (gain: ${this.gain.toFixed(2)}x)`);
        }
    }

    process(inputs, outputs, parameters) {
        const output = outputs[0][0];
        const samplesNeeded = output.length;

        if (this.forceSilence && this.forceSilence > 0) {
            output.fill(0);
            this.forceSilence--;
            return true;
        }

        if (!this.isReady || this.availableSamples < samplesNeeded) {
            output.fill(0);
            this.port.postMessage({ status: "buffer_low" });
            if (this.debug) {
                console.log('[RX Worklet] Buffer underrun, samples needed:', samplesNeeded, 'available:', this.availableSamples);
            }
            return true;
        }

        if (this.sampleRateRatio === 1.0) {
            for (let i = 0; i < samplesNeeded; i++) {
                if (i < this.availableSamples) {
                    output[i] = Math.max(-1, Math.min(1, this.buffer[this.readPos] * this.gain));
                    this.buffer[this.readPos] = 0;
                    this.readPos = (this.readPos + 1) % this.bufferSize;
                } else {
                    output[i] = 0;
                }
            }
            this.availableSamples -= Math.min(samplesNeeded, this.availableSamples);
        } else {
            let inputIndex = 0;
            let inputIndexFloat = 0;
            
            for (let i = 0; i < samplesNeeded; i++) {
                inputIndexFloat = i / this.sampleRateRatio;
                inputIndex = Math.floor(inputIndexFloat);
                
                if (inputIndex < this.availableSamples) {
                    const readPos1 = (this.readPos + inputIndex) % this.bufferSize;
                    const readPos2 = (this.readPos + Math.min(inputIndex + 1, this.availableSamples - 1)) % this.bufferSize;
                    const fraction = inputIndexFloat - inputIndex;
                    const sample = ((1 - fraction) * this.buffer[readPos1] + fraction * this.buffer[readPos2]) * this.gain;
                    output[i] = Math.max(-1, Math.min(1, sample));
                } else {
                    output[i] = 0;
                }
            }
            
            const samplesConsumed = Math.min(Math.ceil(samplesNeeded / this.sampleRateRatio), this.availableSamples);
            this.readPos = (this.readPos + samplesConsumed) % this.bufferSize;
            this.availableSamples -= samplesConsumed;
        }

        if (this.debug && Math.random() < 0.01) {
            const latencyMs = (this.availableSamples / this.inputSampleRate) * 1000;
            console.log(`[RX Worklet] Buffer: ${this.availableSamples} samples, Latency: ${latencyMs.toFixed(1)}ms`);
        }

        return true;
    }
}

registerProcessor("audio-playback-processor", AudioPlaybackProcessor);