import { riderRequestedTorque, updateEngine } from "./engine.js";
import { longitudinalSlip, tireForce } from "./tires.js";

const clamp = (value, low, high) => Math.min(high, Math.max(low, value));

export function initialState(config, options = {}) {
  const speed = options.speed ?? 15;
  const rearOmega = speed / config.rearWheelRadius;
  return {
    time: 0,
    x: 0,
    speed,
    acceleration: 0,
    pitch: options.pitch ?? 0,
    pitchRate: options.pitchRate ?? 0,
    rearOmega,
    frontOmega: speed / config.frontWheelRadius,
    engineRpm: config.engine.idleRpm,
    engineTorque: 0,
    requestedTorque: 0,
    permittedTorque: 0,
    wheelTorque: 0,
    tireForce: 0,
    rearNormal: config.mass * config.gravity * (1 - config.comFromRear / config.wheelbase),
    frontNormal: config.mass * config.gravity * config.comFromRear / config.wheelbase,
    slip: 0,
    frontClearance: 0,
    mode: options.pitch > 0 ? "wheelie" : "ground",
    gear: options.gear ?? 2,
    throttle: options.throttle ?? 0,
    intervention: 0,
    unsafe: false,
    constraintViolations: [],
    delayedTorque: 0,
    commandQueue: []
  };
}

export function totalRatio(config, gear) {
  const index = clamp(Math.round(gear) - 1, 0, config.drivetrain.gearRatios.length - 1);
  return config.drivetrain.primaryRatio * config.drivetrain.gearRatios[index] * config.drivetrain.finalDriveRatio;
}

export function geometryAtPitch(config, pitch) {
  const b = config.comFromRear;
  const h = config.comHeight;
  return {
    comForward: b * Math.cos(pitch) - h * Math.sin(pitch),
    comHeight: config.rearWheelRadius + b * Math.sin(pitch) + h * Math.cos(pitch),
    frontClearance: Math.max(0, config.wheelbase * Math.sin(pitch))
  };
}

/**
 * Integrates the reduced-order hybrid model one semi-implicit Euler step.
 * Ground mode uses quasi-static longitudinal load transfer. Wheelie mode uses
 * pitch moment about the rear contact: I*thetaDDot = m*a*z - m*g*x - c*q.
 */
export function stepVehicle(state, input, config, dt) {
  const ratio = totalRatio(config, input.gear ?? state.gear);
  state.time += dt;
  state.gear = input.gear ?? state.gear;
  state.throttle = clamp(input.throttle ?? state.throttle, 0, 1);
  state.engineRpm = clamp(
    state.rearOmega * ratio * 60 / (2 * Math.PI),
    config.engine.idleRpm,
    config.engine.redlineRpm
  );
  state.requestedTorque = riderRequestedTorque(state.throttle, state.engineRpm, config.engine);

  // Positive authority is reduction-only. Engine braking always remains available to the rider.
  const ceiling = Math.max(0, input.torqueCeiling ?? Number.POSITIVE_INFINITY);
  state.permittedTorque = state.requestedTorque >= 0
    ? Math.min(state.requestedTorque, ceiling)
    : state.requestedTorque;
  state.intervention = Math.max(0, state.requestedTorque - state.permittedTorque);
  updateEngine(state, state.permittedTorque, dt, config.engine);
  state.wheelTorque = state.engineTorque * ratio * config.drivetrain.efficiency;

  const aero = 0.5 * config.airDensity * config.dragCoefficientArea * state.speed * Math.abs(state.speed);
  const rolling = config.rollingResistance * config.mass * config.gravity * Math.sign(state.speed || 1);
  const normalGuess = state.mode === "ground" ? state.rearNormal : config.mass * config.gravity;
  const rawSlip = longitudinalSlip(state.rearOmega, config.rearWheelRadius, state.speed);
  state.slip += (rawSlip - state.slip) * Math.min(1, dt / config.tire.relaxationTime);
  state.slip = clamp(state.slip, -config.tire.maxSlip, config.tire.maxSlip);
  state.tireForce = tireForce(state.slip, normalGuess, config.tire);
  state.acceleration = (state.tireForce - aero - rolling) / config.mass;

  // Rear wheel rotational dynamics closes the force/torque/slip loop.
  const wheelLoss = config.rollingResistance * normalGuess * config.rearWheelRadius;
  const rearAlpha = (state.wheelTorque - state.tireForce * config.rearWheelRadius - wheelLoss) /
    config.rearWheelInertia;
  state.rearOmega = Math.max(0, state.rearOmega + rearAlpha * dt);

  if (state.mode === "ground") {
    const frontNormal = (
      config.mass * config.gravity * config.comFromRear -
      config.mass * state.acceleration * config.comHeight
    ) / config.wheelbase;
    state.frontNormal = Math.max(0, frontNormal);
    state.rearNormal = Math.max(0, config.mass * config.gravity - state.frontNormal);
    state.pitch = 0;
    state.pitchRate = 0;
    state.frontClearance = 0;
    state.frontOmega = Math.max(0, state.speed / config.frontWheelRadius);
    if (frontNormal <= 0 && state.acceleration > 0) {
      state.mode = "wheelie";
      state.frontNormal = 0;
      state.rearNormal = config.mass * config.gravity;
    }
  } else {
    const geometry = geometryAtPitch(config, state.pitch);
    const pitchMoment = config.mass * state.acceleration * geometry.comHeight -
      config.mass * config.gravity * geometry.comForward -
      config.pitchDamping * state.pitchRate;
    const pitchAcceleration = pitchMoment / config.pitchInertia;
    state.pitchRate += pitchAcceleration * dt;
    state.pitch += state.pitchRate * dt;
    state.frontNormal = 0;
    state.rearNormal = config.mass * config.gravity;
    state.frontClearance = geometryAtPitch(config, state.pitch).frontClearance;
    // Small bearing drag while airborne.
    state.frontOmega = Math.max(0, state.frontOmega * Math.exp(-0.015 * dt));
    if (state.pitch <= 0 && state.pitchRate < 0) {
      state.pitch = 0;
      state.frontClearance = 0;
      state.pitchRate = Math.max(0, -state.pitchRate * config.restitution);
      state.mode = "ground";
      state.frontOmega = state.speed / config.frontWheelRadius;
    }
  }

  state.speed = Math.max(0, state.speed + state.acceleration * dt);
  state.x += state.speed * dt;
  state.unsafe = state.pitch >= config.safety.maxPitch || Math.abs(state.pitchRate) >= config.safety.maxPitchRate;
  state.constraintViolations = [];
  if (state.permittedTorque > Math.max(0, state.requestedTorque) + 1e-9) {
    state.constraintViolations.push("rider-authority");
  }
  if (Math.abs(state.tireForce) > config.tire.friction * state.rearNormal + 1e-6) {
    state.constraintViolations.push("traction");
  }
  if (state.unsafe) state.constraintViolations.push("safety-envelope");
  return state;
}

