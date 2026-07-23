import { Controller, timedCompute } from "./base.js";

const clamp = (value, low, high) => Math.min(high, Math.max(low, value));

/**
 * Lightweight nonlinear predictive controller using direct shooting over a
 * discrete set of admissible reduction sequences. It is intentionally
 * dependency-free and transparent; it is not a replacement for a validated
 * real-time NMPC stack such as CasADi/acados/MATMPC.
 */
export class PredictiveController extends Controller {
  constructor(options = {}) {
    super(options);
    this.horizon = options.horizon ?? 0.6;
    this.nodes = options.nodes ?? 15;
    this.maxCeilingRate = options.maxCeilingRate ?? 650;
    this.lastCeiling = Number.POSITIVE_INFINITY;
    this.solverFailures = 0;
    this.prediction = [];
  }

  reset() {
    super.reset();
    this.lastCeiling = Number.POSITIVE_INFINITY;
    this.solverFailures = 0;
    this.prediction = [];
  }

  compute(observation, context) {
    return timedCompute(this, () => {
      const request = Math.max(0, observation.requestedTorque);
      if (!Number.isFinite(observation.pitch) || !Number.isFinite(observation.pitchRate)) {
        this.solverFailures += 1;
        return { torqueCeiling: 0, status: "solver-fallback", solverFailed: true, prediction: [] };
      }
      if (observation.mode === "ground") {
        this.lastCeiling = request;
        return { torqueCeiling: request, status: "armed", solverFailed: false, prediction: [] };
      }
      const previous = Number.isFinite(this.lastCeiling) ? this.lastCeiling : request;
      const maxStep = this.maxCeilingRate * context.controllerDt;
      const candidates = [0, 0.12, 0.25, 0.4, 0.58, 0.75, 0.9, 1]
        .map(fraction => request * fraction)
        .map(value => clamp(value, Math.max(0, previous - maxStep), Math.min(request, previous + maxStep)));
      let best = null;
      for (const ceiling of new Set(candidates)) {
        const trial = this.predict(observation, context, ceiling, previous);
        if (!best || trial.cost < best.cost) best = trial;
      }
      if (!best || !Number.isFinite(best.cost)) {
        this.solverFailures += 1;
        this.lastCeiling = 0;
        return { torqueCeiling: 0, status: "solver-fallback", solverFailed: true, prediction: [] };
      }
      this.lastCeiling = best.ceiling;
      this.prediction = best.trajectory;
      const insufficient = observation.pitch < context.targetPitch && request <= 0;
      return {
        torqueCeiling: best.ceiling,
        status: insufficient ? "insufficient-rider-demand" : best.ceiling < request - 1e-6 ? "optimizing" : "armed",
        solverFailed: false,
        prediction: best.trajectory,
        objective: best.cost
      };
    });
  }

  predict(observation, context, ceiling, previous) {
    const { config, targetPitch } = context;
    const step = this.horizon / this.nodes;
    let pitch = observation.pitch;
    let rate = observation.pitchRate;
    let actualTorque = observation.engineTorque ?? ceiling;
    let cost = 0;
    const trajectory = [];
    for (let index = 0; index < this.nodes; index += 1) {
      actualTorque += (ceiling - actualTorque) * Math.min(1, step / config.engine.responseTime);
      const ratio = config.drivetrain.primaryRatio * config.drivetrain.gearRatios[(observation.gear ?? 2) - 1] *
        config.drivetrain.finalDriveRatio * config.drivetrain.efficiency;
      const driveForce = clamp(actualTorque * ratio / config.rearWheelRadius,
        0, config.tire.friction * config.mass * config.gravity);
      const acceleration = driveForce / config.mass - config.rollingResistance * config.gravity;
      const x = config.comFromRear * Math.cos(pitch) - config.comHeight * Math.sin(pitch);
      const z = config.rearWheelRadius + config.comFromRear * Math.sin(pitch) + config.comHeight * Math.cos(pitch);
      const angularAcceleration = (config.mass * acceleration * z - config.mass * config.gravity * x -
        config.pitchDamping * rate) / config.pitchInertia;
      rate += angularAcceleration * step;
      pitch += rate * step;
      const error = pitch - targetPitch;
      const intervention = Math.max(0, observation.requestedTorque - ceiling);
      const slew = ceiling - previous;
      cost += 1400 * error * error + 70 * rate * rate + 0.002 * intervention * intervention + 0.004 * slew * slew;
      if (pitch > config.safety.warningPitch) cost += 1e5 * (pitch - config.safety.warningPitch + 0.01) ** 2;
      if (pitch < -0.02) cost += 2e4 * pitch * pitch;
      trajectory.push({ time: observation.time + (index + 1) * step, pitch, pitchRate: rate });
    }
    return { ceiling, cost, trajectory };
  }
}

