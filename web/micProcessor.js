// AudioWorklet processor for sBitx microphone processing
// This runs in a separate thread for better performance

class MicAudioProcessor extends AudioWorkletProcessor {
  constructor() {
    super();
    this.buffer = [];
    this.sampleRateLogged = false;
    
    // Audio processing parameters
    this.targetRate = 8000; // Target 8kHz output
    this.gainReduction = 0.15; // Reduce to 15% to prevent clipping
    
    // Low-pass filter state
    this.lpfLastSample = 0;
    this.lpfAlpha = 0.7; // Higher alpha for gentler filtering (0-1)
    
    // High-pass filter state
    this.hpfLastSample = 0;
    this.hpfAlpha = 0.2; // Stronger high-pass effect
  }
  
  process(inputs, outputs, parameters) {
    // Get mono input from first channel
    const input = inputs[0][0];
    
    // Skip processing if no input or not enough samples
    if (!input || input.length === 0) {
      return true; // Keep processor alive
    }
    
    // Calculate downsampling ratio based on actual sample rate
    const ratio = Math.floor(sampleRate / this.targetRate);
    const validRatio = ratio < 1 ? 1 : ratio;
    
    // Log sample rate once
    if (!this.sampleRateLogged) {
      console.log("AudioWorklet using sample rate: " + sampleRate + "Hz, ratio: " + validRatio);
      this.sampleRateLogged = true;
    }
    
    // Downsample the audio
    const newLength = Math.floor(input.length / validRatio);
    const downsampledData = new Float32Array(newLength);
    
    // Simple downsampling with gain reduction
    for (let i = 0; i < newLength; i++) {
      downsampledData[i] = input[i * validRatio] * this.gainReduction;
    }
    
    // Apply high-frequency boost for clarity
    const boostedData = this.applyHighFrequencyBoost(downsampledData);
    
    // Apply low-pass filter to smooth any harshness
    const filteredData = this.applyLowPassFilter(boostedData);
    
    // Send processed audio to main thread
    this.port.postMessage({
      audioData: filteredData
    });
    
    return true; // Keep processor alive
  }
  
  applyLowPassFilter(samples) {
    // Very gentle first-order low-pass filter
    const filtered = new Float32Array(samples.length);
    let lastSample = this.lpfLastSample;
    
    for (let i = 0; i < samples.length; i++) {
      filtered[i] = this.lpfAlpha * samples[i] + (1 - this.lpfAlpha) * lastSample;
      lastSample = filtered[i];
    }
    
    this.lpfLastSample = lastSample; // Save state between calls
    return filtered;
  }
  
  applyHighFrequencyBoost(samples) {
    // Strong high-frequency boost for better clarity
    const filtered = new Float32Array(samples.length);
    let lastSample = this.hpfLastSample;
    
    for (let i = 0; i < samples.length; i++) {
      // High-pass filter
      const highPass = samples[i] - lastSample;
      lastSample = samples[i];
      
      // Add high-pass signal back to the original with strong boost
      filtered[i] = samples[i] + (highPass * 1.5); // Boost high frequencies by 150%
      
      // Limit to prevent clipping
      if (filtered[i] > 1.0) filtered[i] = 1.0;
      if (filtered[i] < -1.0) filtered[i] = -1.0;
    }
    
    this.hpfLastSample = lastSample; // Save state between calls
    return filtered;
  }
  
  // calculateAudioLevel function removed as we no longer need visual feedback
}

// Register the processor with a name that will be used to create it
registerProcessor('mic-audio-processor', MicAudioProcessor);
