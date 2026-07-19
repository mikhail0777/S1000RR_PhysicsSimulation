import test from "node:test";
import assert from "node:assert/strict";
import { scenarios } from "../src/simulation/scenarios.js";
import { runSimulation } from "../src/simulation/runner.js";
import { evaluateRun } from "../src/analysis/metrics.js";

test("simulation is deterministic for a fixed seed", () => {
  const a = runSimulation(scenarios.noise, { duration: 2, seed: 123 });
  const b = runSimulation(scenarios.noise, { duration: 2, seed: 123 });
  const physical = history => history.map(({ computeMs: _timing, ...sample }) => sample);
  assert.deepEqual(physical(a.history), physical(b.history));
});

test("every controller obeys rider positive-torque ceiling", () => {
  for (const controller of ["none", "threshold", "pid", "delay-aware", "nmpc"]) {
    const run = runSimulation(scenarios.excessive, { controller, duration: 3 });
    for (const state of run.history) {
      assert.ok(state.permittedTorque <= Math.max(0, state.requestedTorque) + 1e-9);
    }
  }
});

test("timestep refinement preserves the gross physical outcome", () => {
  const coarse = runSimulation(scenarios.unassisted, { dt: 0.002, duration: 3 });
  const fine = runSimulation(scenarios.unassisted, { dt: 0.001, duration: 3 });
  assert.equal(coarse.finalState.mode, fine.finalState.mode);
  assert.ok(Math.abs(coarse.finalState.speed - fine.finalState.speed) < 0.8);
  assert.ok(Math.abs(coarse.finalState.pitch - fine.finalState.pitch) < 0.08);
});

test("analysis distinguishes lift and reports finite metrics", () => {
  const run = runSimulation(scenarios.ideal, { duration: 4 });
  const metrics = evaluateRun(run);
  assert.equal(metrics.frontLifted, true);
  assert.ok(Number.isFinite(metrics.pitchRmseDeg));
  assert.ok(metrics.classification.length > 0);
});

