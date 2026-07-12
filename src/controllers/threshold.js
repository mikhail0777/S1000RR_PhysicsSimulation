import { Controller, timedCompute } from "./base.js";

export class ThresholdController extends Controller {
  constructor(options = {}) {
    super(options);
    this.threshold = options.threshold ?? 0.17;
    this.band = options.band ?? 0.07;
  }

  compute(observation) {
    return timedCompute(this, () => {
      const positiveRequest = Math.max(0, observation.requestedTorque);
      const reduction = Math.min(1, Math.max(0, (observation.pitch - this.threshold) / this.band));
      return {
        torqueCeiling: positiveRequest * (1 - reduction),
        status: reduction > 0 ? "limiting" : "armed"
      };
    });
  }
}

