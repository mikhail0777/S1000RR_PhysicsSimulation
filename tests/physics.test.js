import test from "node:test";
import assert from "node:assert/strict";
import { motorcycleConfig } from "../config/motorcycle.js";
import { initialState, stepVehicle } from "../src/physics/vehicle.js";
import { tireForce } from "../src/physics/tires.js";

const cloneConfig = () => structuredClone(motorcycleConfig);

test("zero throttle cannot create unexplained positive acceleration", () => {
  const config = cloneConfig();
  const state = initialState(config, { speed: 20, gear: 2 });
  for (let i = 0; i < 500; i += 1) stepVehicle(state, { throttle: 0, gear: 2 }, config, 0.001);
  assert.ok(state.speed < 20, `speed should fall, got ${state.speed}`);
});

test("tire law respects friction circle longitudinal bound", () => {
  for (const slip of [-10, -1, 0, 1, 10]) {
    assert.ok(Math.abs(tireForce(slip, 1400, motorcycleConfig.tire)) <= 1400 * motorcycleConfig.tire.friction);
  }
});

test("reduction-only ceiling cannot exceed rider request", () => {
  const config = cloneConfig();
  const state = initialState(config, { speed: 18, gear: 1 });
  for (let i = 0; i < 100; i += 1) {
    stepVehicle(state, { throttle: 0.7, gear: 1, torqueCeiling: 1e6 }, config, 0.001);
    assert.ok(state.permittedTorque <= state.requestedTorque + 1e-12);
  }
});

test("sufficient first-gear drive transfers load and lifts front contact", () => {
  const config = cloneConfig();
  const state = initialState(config, { speed: 12, gear: 1 });
  for (let i = 0; i < 2500 && state.mode === "ground"; i += 1) {
    stepVehicle(state, { throttle: 1, gear: 1 }, config, 0.001);
  }
  assert.equal(state.mode, "wheelie");
  assert.equal(state.frontNormal, 0);
});

test("gravity returns an unpowered lifted front wheel to ground", () => {
  const config = cloneConfig();
  const state = initialState(config, { speed: 15, gear: 3, pitch: 0.25, pitchRate: 0 });
  for (let i = 0; i < 5000 && state.mode === "wheelie"; i += 1) {
    stepVehicle(state, { throttle: 0, gear: 3, torqueCeiling: 0 }, config, 0.001);
  }
  assert.equal(state.mode, "ground");
  assert.equal(state.frontClearance, 0);
});

test("mass increase reduces acceleration under the same tire force state", () => {
  const light = cloneConfig();
  const heavy = cloneConfig();
  heavy.mass *= 1.35;
  light.tire.friction = 10;
  heavy.tire.friction = 10;
  light.rollingResistance = 0;
  heavy.rollingResistance = 0;
  const a = initialState(light, { speed: 18, gear: 2 });
  const b = initialState(heavy, { speed: 18, gear: 2 });
  a.rearOmega *= 1.04;
  b.rearOmega *= 1.04;
  a.slip = 0.04;
  b.slip = 0.04;
  stepVehicle(a, { throttle: 0.5, gear: 2 }, light, 0.001);
  stepVehicle(b, { throttle: 0.5, gear: 2 }, heavy, 0.001);
  assert.ok(a.acceleration > b.acceleration);
});

