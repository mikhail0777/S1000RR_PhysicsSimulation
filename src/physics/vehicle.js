import { riderRequestedTorque, updateEngine } from "./engine.js";
import { longitudinalSlip, tireForce } from "./tires.js";

const clamp = (value, low, high) => Math.min(high, Math.max(low, value));

export function initialState(config, options = {}) {
  const speed = options.speed ?? 15;
  const rearOmega = speed / config.rearWheelRadius;
  const state = {
    time: 0,
    x: 0,
    speed,
    acceleration: 0,
    pitch: options.pitch ?? 0,
    pitchRate: options.pitchRate ?? 0,
    rearOmega,
    frontOmega: speed / config.frontWheelRadius,
    rearWheelAngle: 0,
    frontWheelAngle: 0,
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
    commandQueue: [],
    drivetrainWork: 0,
    aerodynamicLoss: 0,
    rollingLoss: 0,
    slipLoss: 0,
    impactLoss: 0,
    mechanicalEnergy: 0,
    energyResidual: 0
  };
  state.mechanicalEnergy = mechanicalEnergy(state, config);
  state.initialMechanicalEnergy = state.mechanicalEnergy;
  return state;
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

export function calculateNormalLoads(config, acceleration) {
  const front = (config.mass * config.gravity * config.comFromRear -
    config.mass * acceleration * config.comHeight) / config.wheelbase;
  return {
    front: Math.max(0, front),
    rear: Math.max(0, config.mass * config.gravity - Math.max(0, front)),
    unconstrainedFront: front
  };
}

export function mechanicalEnergy(state, config) {
  const relativeHeight = config.comFromRear * Math.sin(state.pitch) +
    config.comHeight * (Math.cos(state.pitch) - 1);
  return 0.5 * config.mass * state.speed ** 2 +
    0.5 * config.rearWheelInertia * state.rearOmega ** 2 +
    0.5 * config.frontWheelInertia * state.frontOmega ** 2 +
    0.5 * config.pitchInertia * state.pitchRate ** 2 +
    config.mass * config.gravity * relativeHeight;
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
    const loads = calculateNormalLoads(config, state.acceleration);
    const frontNormal = loads.unconstrainedFront;
    state.frontNormal = loads.front;
    state.rearNormal = loads.rear;
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
      const preImpactEnergy = mechanicalEnergy(state, config);
      state.pitch = 0;
      state.frontClearance = 0;
      state.pitchRate = Math.max(0, -state.pitchRate * config.restitution);
      state.mode = "ground";
      state.frontOmega = state.speed / config.frontWheelRadius;
      state.impactLoss += Math.max(0, preImpactEnergy - mechanicalEnergy(state, config));
    }
  }

  state.speed = Math.max(0, state.speed + state.acceleration * dt);
  state.x += state.speed * dt;
  state.rearWheelAngle = (state.rearWheelAngle + state.rearOmega * dt) % (2 * Math.PI);
  state.frontWheelAngle = (state.frontWheelAngle + state.frontOmega * dt) % (2 * Math.PI);
  state.drivetrainWork += state.wheelTorque * state.rearOmega * dt;
  state.aerodynamicLoss += Math.max(0, aero * state.speed) * dt;
  state.rollingLoss += (Math.abs(rolling * state.speed) + Math.abs(wheelLoss * state.rearOmega)) * dt;
  state.slipLoss += Math.max(0, state.tireForce *
    (state.rearOmega * config.rearWheelRadius - state.speed)) * dt;
  state.mechanicalEnergy = mechanicalEnergy(state, config);
  const explainedEnergy = state.initialMechanicalEnergy + state.drivetrainWork -
    state.aerodynamicLoss - state.rollingLoss - state.slipLoss - state.impactLoss;
  state.energyResidual = state.mechanicalEnergy - explainedEnergy;
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

