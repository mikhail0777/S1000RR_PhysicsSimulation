import test from "node:test";
import assert from "node:assert/strict";
import { ReductionOnlyPidController } from "../src/controllers/pid.js";
import { ThresholdController } from "../src/controllers/threshold.js";

const context = { targetPitch: 0.2, controllerDt: 0.01 };

test("threshold limiter leaves request available below threshold", () => {
  const controller = new ThresholdController({ threshold: 0.18 });
  const result = controller.compute({ pitch: 0.1, requestedTorque: 80 });
  assert.equal(result.torqueCeiling, 80);
});

test("PID reduces torque for overshoot and positive pitch rate", () => {
  const controller = new ReductionOnlyPidController();
  const result = controller.compute({ pitch: 0.24, pitchRate: 0.3, requestedTorque: 90 }, context);
  assert.ok(result.torqueCeiling < 90);
  assert.ok(result.torqueCeiling >= 0);
});

test("PID cannot create positive torque when rider closes throttle", () => {
  const controller = new ReductionOnlyPidController();
  const result = controller.compute({ pitch: 0.05, pitchRate: -0.2, requestedTorque: -12 }, context);
  assert.equal(result.torqueCeiling, 0);
  assert.equal(result.status, "insufficient-rider-demand");
});

