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
      if (observation.mode === "ground") {
        return { torqueCeiling: positiveRequest, status: "armed", error, saturated: false };
      }

      // Wheelie equilibrium feedforward: choose drive acceleration such that
      // m*a*z_com balances m*g*x_com, then map rear force back to engine torque.
      let feedforward = positiveRequest;
      if (context.config) {
        const c = context.config;
        const x = c.comFromRear * Math.cos(predictedPitch) - c.comHeight * Math.sin(predictedPitch);
        const z = c.rearWheelRadius + c.comFromRear * Math.sin(predictedPitch) + c.comHeight * Math.cos(predictedPitch);
        const equilibriumAcceleration = c.gravity * x / Math.max(0.1, z);
        const force = c.mass * (equilibriumAcceleration + c.rollingResistance * c.gravity);
        const gearIndex = Math.max(0, Math.min(c.drivetrain.gearRatios.length - 1, (observation.gear ?? 2) - 1));
        const ratio = c.drivetrain.primaryRatio * c.drivetrain.gearRatios[gearIndex] *
          c.drivetrain.finalDriveRatio * c.drivetrain.efficiency;
        feedforward = Math.max(0, force * c.rearWheelRadius / ratio);
      }
      // A bounded reference-rate governor avoids suppressing all upward motion
      // immediately after lift-off; the desired rate smoothly tends to zero at target.
      const referenceRate = Math.min(1.2, Math.max(-1.2, 3.5 * (context.targetPitch - predictedPitch)));
      const feedback = this.kp * error + this.kd * (observation.pitchRate - referenceRate) + this.ki * this.integral;
      const unconstrainedCeiling = feedforward - feedback;
      const ceiling = Math.min(positiveRequest, Math.max(0, unconstrainedCeiling));
      const saturated = ceiling !== unconstrainedCeiling;

      // Conditional integration prevents windup at either one-sided authority boundary.
      if (!saturated || (ceiling === 0 && error < 0) || (ceiling === positiveRequest && error > 0)) {
        this.integral = Math.min(this.integralLimit, Math.max(-this.integralLimit,
          this.integral + error * context.controllerDt));
      }
      const cannotAdd = predictedPitch < context.targetPitch && positiveRequest + 1e-6 < unconstrainedCeiling;
      return {
        torqueCeiling: ceiling,
        status: cannotAdd ? "insufficient-rider-demand" : ceiling < positiveRequest ? "limiting" : "armed",
        error,
        saturated,
        equilibriumTorque: feedforward,
        referenceRate
      };
    });
  }
}

export class DelayAwareController extends ReductionOnlyPidController {
  constructor(options = {}) {
    super({ ...options, predictSeconds: options.predictSeconds ?? 0.12 });
  }
}

