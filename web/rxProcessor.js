class AudioPlaybackProcessor extends AudioWorkletProcessor {
    constructor(options) {
        super();
        
        // Get sample rate information from options if available
        const processorOptions = options.processorOptions || {};
        
        this.bufferSize = 4096; // Increased buffer size for smoother playback
        this.buffer = new Float32Array(this.bufferSize);
        this.writePos = 0;
        this.readPos = 0;
        this.availableSamples = 0;
        this.isReady = false;
        this.minBufferThreshold = 1536; // ~32ms at 16kHz
        
        // The sBitx system uses 96kHz internally, decimated to 16kHz for the browser
        this.inputSampleRate = processorOptions.inputSampleRate || 16000;
        this.outputSampleRate = processorOptions.outputSampleRate || sampleRate;
        
        // Calculate the sample rate ratio for resampling
        this.sampleRateRatio = this.outputSampleRate / this.inputSampleRate;
        
        // Debug flag - set to false to hide buffer messages
        this.debug = processorOptions.debug || false;
        
        // Initialize the force silence counter (used during TX transition)
        this.forceSilence = 0;
        
        // Log the actual AudioContext sample rate when first sample arrives
        this.sampleRateLogged = false;

        this.port.onmessage = (event) => {
            // Check if this is a command message
            if (event.data && typeof event.data === 'object' && event.data.command) {
                // Handle command messages
                if (event.data.command === 'clear_buffer') {
                    // Completely clear the buffer and reset all state
                    this.buffer = new Float32Array(this.bufferSize); // Create a fresh buffer
                    this.writePos = 0;
                    this.readPos = 0;
                    this.availableSamples = 0;
                    this.isReady = false;
                    
                    // Force silence output for the next few process calls
                    this.forceSilence = 30; // Force silence for ~30 process calls (about 300ms at typical rates)
                    
                    // Send acknowledgment
                    this.port.postMessage({ status: "buffer_cleared" });
                    
                    if (this.debug) {
                        console.log('[RX Worklet] Buffer completely cleared and reset');
                    }
                    return;
                }
                return;
            }
            
            // Handle regular audio data
            if (this.availableSamples >= this.bufferSize) {
                this.port.postMessage({ status: "buffer_full" });
                return;
            }
            
            // Log the actual AudioContext sample rate once if debug is enabled
            if (!this.sampleRateLogged && this.debug) {
                console.log(`[RX Worklet] Audio context sample rate: ${sampleRate}Hz, Input rate: ${this.inputSampleRate}Hz`);
                this.sampleRateLogged = true;
            }

            const int16Data = new Int16Array(event.data);
            const float32Data = new Float32Array(int16Data.length);

            // Log sample statistics
            let min = 32767, max = -32768, sum = 0;
            for (let i = 0; i < int16Data.length; i++) {
                if (int16Data[i] < min) min = int16Data[i];
                if (int16Data[i] > max) max = int16Data[i];
                sum += int16Data[i];
            }
            const mean = sum / int16Data.length;
            
            // Normalize samples to float32 (-1.0 to 1.0 range)
            for (let i = 0; i < int16Data.length; i++) {
                // Standard Int16 normalization
                let sample = int16Data[i] / 32768.0;
                
                // Apply very slight DC offset correction if needed
                if (Math.abs(mean) > 100) { // Only correct if there's a significant DC offset
                    sample = sample - (mean / 32768.0 * 0.5); // Apply 50% correction to avoid over-correction
                }
                
                float32Data[i] = Math.max(-1.0, Math.min(1.0, sample)); // Ensure we stay in valid range
            }

            //console.log("Received samples:", int16Data.length);

            // Write to circular buffer
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

    process(inputs, outputs, parameters) {
        const output = outputs[0][0];
        const samplesNeeded = output.length;

        // Check if we're in forced silence mode (after buffer clear)
        if (this.forceSilence && this.forceSilence > 0) {
            output.fill(0);
            this.forceSilence--;
            return true;
        }

        if (!this.isReady || this.availableSamples < samplesNeeded) {
            // Buffer underrun - fill with silence
            output.fill(0);
            this.port.postMessage({ status: "buffer_low" });
            return true;
        }

        // Use the pre-calculated sample rate ratio from the constructor
        // This is critical for maintaining the correct pitch
        
        if (this.sampleRateRatio === 1.0) {
            // No resampling needed - direct copy
            for (let i = 0; i < samplesNeeded; i++) {
                if (i < this.availableSamples) {
                    output[i] = Math.max(-1, Math.min(1, this.buffer[this.readPos]));
                    this.buffer[this.readPos] = 0;
                    this.readPos = (this.readPos + 1) % this.bufferSize;
                } else {
                    output[i] = 0; // Pad with silence if we run out of samples
                }
            }
            this.availableSamples -= Math.min(samplesNeeded, this.availableSamples);
        } else {
            // Resampling needed - linear interpolation for simplicity and low CPU usage
            let inputIndex = 0;
            let inputIndexFloat = 0;
            
            for (let i = 0; i < samplesNeeded; i++) {
                // Calculate the exact position in the input buffer
                inputIndexFloat = i / this.sampleRateRatio;
                inputIndex = Math.floor(inputIndexFloat);
                
                if (inputIndex < this.availableSamples) {
                    // Get the two adjacent samples for interpolation
                    const readPos1 = (this.readPos + inputIndex) % this.bufferSize;
                    const readPos2 = (this.readPos + Math.min(inputIndex + 1, this.availableSamples - 1)) % this.bufferSize;
                    
                    // Calculate the fractional part for interpolation
                    const fraction = inputIndexFloat - inputIndex;
                    
                    // Linear interpolation between the two samples
                    output[i] = (1 - fraction) * this.buffer[readPos1] + fraction * this.buffer[readPos2];
                    
                    // Ensure we stay in the valid range
                    output[i] = Math.max(-1, Math.min(1, output[i]));
                } else {
                    output[i] = 0; // Pad with silence if we run out of samples
                }
            }
            
            // Update read position and available samples count
            const samplesConsumed = Math.min(Math.ceil(samplesNeeded / this.sampleRateRatio), this.availableSamples);
            this.readPos = (this.readPos + samplesConsumed) % this.bufferSize;
            this.availableSamples -= samplesConsumed;
        }

        // Log estimated latency (occasionally) only if debug is enabled
        if (this.debug && Math.random() < 0.01) { // Only log about 1% of the time to reduce console spam
            const latencyMs = (this.availableSamples / this.inputSampleRate) * 1000;
            console.log(`[RX Worklet] Buffer: ${this.availableSamples} samples, Latency: ${latencyMs.toFixed(1)}ms`);
        }

        return true;
    }
}

registerProcessor("audio-playback-processor", AudioPlaybackProcessor);