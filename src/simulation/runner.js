import { motorcycleConfig } from "../../config/motorcycle.js";
import { createController } from "../controllers/index.js";
import { initialState, stepVehicle } from "../physics/vehicle.js";
import { SensorModel } from "../sensors/model.js";

export function stateSnapshot(state, controllerOutput, targetPitch) {
  return {
    time: state.time, x: state.x, speed: state.speed, acceleration: state.acceleration,
    pitch: state.pitch, pitchRate: state.pitchRate, targetPitch,
    throttle: state.throttle, requestedTorque: state.requestedTorque,
    permittedTorque: state.permittedTorque, engineTorque: state.engineTorque,
    wheelTorque: state.wheelTorque, engineRpm: state.engineRpm,
    rearOmega: state.rearOmega, frontOmega: state.frontOmega,
    rearWheelAngle: state.rearWheelAngle, frontWheelAngle: state.frontWheelAngle,
    tireForce: state.tireForce, slip: state.slip,
    frontNormal: state.frontNormal, rearNormal: state.rearNormal,
    frontClearance: state.frontClearance, mode: state.mode,
    intervention: state.intervention, unsafe: state.unsafe,
    controllerStatus: controllerOutput.status,
    computeMs: controllerOutput.computeMs ?? 0,
    solverFailed: controllerOutput.solverFailed ?? false,
    mechanicalEnergy: state.mechanicalEnergy,
    energyResidual: state.energyResidual,
    violations: [...state.constraintViolations]
  };
}

export function runSimulation(scenario, options = {}) {
  const config = structuredClone(options.config ?? motorcycleConfig);
  if (scenario.friction) config.tire.friction = scenario.friction;
  if (scenario.engineDelay !== undefined) config.engine.transportDelay = scenario.engineDelay;
  if (scenario.engineResponseTime !== undefined) config.engine.responseTime = scenario.engineResponseTime;
  if (scenario.comFromRear !== undefined) config.comFromRear = scenario.comFromRear;
  if (scenario.comHeight !== undefined) config.comHeight = scenario.comHeight;
  const dt = options.dt ?? 0.002;
  const controllerDt = options.controllerDt ?? 0.01;
  const controllerName = options.controller ?? scenario.controller ?? "none";
  const controller = createController(controllerName, options.controllerOptions);
  const sensor = new SensorModel({ ...scenario.sensor, ...options.sensor, seed: options.seed ?? 42 });
  const state = initialState(config, {
    speed: scenario.initialSpeed ?? 15,
    gear: scenario.gear ?? 2,
    pitch: scenario.initialPitch ?? 0
  });
  const history = [];
  let output = { torqueCeiling: Number.POSITIVE_INFINITY, status: "initializing", computeMs: 0 };
  let nextControl = 0;
  let nextRecord = 0;
  const duration = options.duration ?? scenario.duration;
  const targetPitch = options.targetPitch ?? scenario.targetPitch;
  let terminatedReason = "duration";

  while (state.time < duration - dt / 2) {
    const throttle = options.throttle ?? scenario.throttle(state.time);
    if (state.time + 1e-12 >= nextControl) {
      const observation = sensor.sample(state);
      observation.engineTorque = state.engineTorque;
      observation.gear = scenario.gear;
      const faulted = scenario.faultWindow && state.time >= scenario.faultWindow[0] && state.time <= scenario.faultWindow[1];
      if (faulted) {
        output = { torqueCeiling: Number.POSITIVE_INFINITY, status: "command-fault", computeMs: 0 };
      } else {
        output = controller.compute(observation, { targetPitch, controllerDt, config });
      }
      nextControl += controllerDt;
    }
    stepVehicle(state, { throttle, gear: scenario.gear, torqueCeiling: output.torqueCeiling }, config, dt);
    if (state.time + 1e-12 >= nextRecord) {
      history.push(stateSnapshot(state, output, targetPitch));
      nextRecord += options.recordDt ?? 0.01;
    }
    if (state.unsafe) {
      terminatedReason = "safety-envelope";
      if (history.at(-1)?.time !== state.time) history.push(stateSnapshot(state, output, targetPitch));
      break;
    }
  }
  return { scenario, controllerName, config, dt, controllerDt, history, finalState: state, terminatedReason };
}

