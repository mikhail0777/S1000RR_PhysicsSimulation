import { Controller, timedCompute } from "./base.js";

export class ReductionOnlyPidController extends Controller {
  constructor(options = {}) {
    super(options);
    this.kp = options.kp ?? 180;
    this.kd = options.kd ?? 48;
    this.ki = options.ki ?? 18;
    this.integralLimit = options.integralLimit ?? 0.25;
    this.predictSeconds = options.predictSeconds ?? 0;
    this.integral = 0;
  }

  reset() {
    super.reset();
    this.integral = 0;
  }

  compute(observation, context) {
    return timedCompute(this, () => {
      const predictedPitch = observation.pitch + observation.pitchRate * this.predictSeconds;
      const error = predictedPitch - context.targetPitch;
      const positiveRequest = Math.max(0, observation.requestedTorque);
      const unconstrainedReduction = this.kp * error + this.kd * observation.pitchRate + this.ki * this.integral;
      const reduction = Math.min(positiveRequest, Math.max(0, unconstrainedReduction));
      const saturated = reduction !== unconstrainedReduction;

      // Conditional integration prevents windup at either one-sided authority boundary.
      if (!saturated || (reduction === 0 && error > 0) || (reduction === positiveRequest && error < 0)) {
        this.integral = Math.min(this.integralLimit, Math.max(-this.integralLimit,
          this.integral + error * context.controllerDt));
      }
      const cannotAdd = predictedPitch < context.targetPitch && observation.requestedTorque <= 0;
      return {
        torqueCeiling: positiveRequest - reduction,
        status: cannotAdd ? "insufficient-rider-demand" : reduction > 0 ? "limiting" : "armed",
        error,
        saturated
      };
    });
  }
}

export class DelayAwareController extends ReductionOnlyPidController {
  constructor(options = {}) {
    super({ ...options, predictSeconds: options.predictSeconds ?? 0.12 });
  }
}

