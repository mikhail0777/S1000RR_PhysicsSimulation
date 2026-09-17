function mulberry32(seed) {
  return function random() {
    let t = seed += 0x6D2B79F5;
    t = Math.imul(t ^ t >>> 15, t | 1);
    t ^= t + Math.imul(t ^ t >>> 7, t | 61);
    return ((t ^ t >>> 14) >>> 0) / 4294967296;
  };
}

function gaussian(random) {
  const u = Math.max(1e-12, random());
  const v = random();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

export class SensorModel {
  constructor(options = {}) {
    this.options = {
      latency: options.latency ?? 0,
      pitchNoise: options.pitchNoise ?? 0,
      rateNoise: options.rateNoise ?? 0,
      pitchBias: options.pitchBias ?? 0,
      quantization: options.quantization ?? 0,
      dropoutProbability: options.dropoutProbability ?? 0,
      timestampJitter: options.timestampJitter ?? 0
    };
    this.random = mulberry32(options.seed ?? 42);
    this.history = [];
    this.last = null;
  }

  sample(state) {
    const snapshot = {
      time: state.time,
      pitch: state.pitch,
      pitchRate: state.pitchRate,
      speed: state.speed,
      engineRpm: state.engineRpm,
      requestedTorque: state.requestedTorque,
      frontClearance: state.frontClearance,
      slip: state.slip,
      mode: state.mode
    };
    this.history.push(snapshot);
    const jitter = (this.random() * 2 - 1) * this.options.timestampJitter;
    const targetTime = state.time - this.options.latency - jitter;
    let delayed = this.history[0];
    for (const item of this.history) {
      if (item.time <= targetTime) delayed = item;
      else break;
    }
    this.history = this.history.filter(item => item.time >= targetTime - 0.5);
    if (this.random() < this.options.dropoutProbability && this.last) return { ...this.last, dropout: true };
    const quantize = value => this.options.quantization > 0
      ? Math.round(value / this.options.quantization) * this.options.quantization : value;
    this.last = {
      ...delayed,
      pitch: quantize(delayed.pitch + this.options.pitchBias + gaussian(this.random) * this.options.pitchNoise),
      pitchRate: quantize(delayed.pitchRate + gaussian(this.random) * this.options.rateNoise),
      dropout: false
    };
    return this.last;
  }
}

