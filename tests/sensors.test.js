import test from "node:test";
import assert from "node:assert/strict";
import { SensorModel } from "../src/sensors/model.js";

const state = (time, pitch) => ({ time, pitch, pitchRate: pitch * 2, speed: 10,
  engineRpm: 4000, requestedTorque: 40, frontClearance: 0, slip: 0, mode: "ground" });

test("sensor latency returns the latest sample at or before delayed time", () => {
  const sensor = new SensorModel({ latency: 0.1, seed: 1 });
  sensor.sample(state(0, 0));
  sensor.sample(state(0.05, 0.1));
  const delayed = sensor.sample(state(0.1, 0.2));
  assert.equal(delayed.pitch, 0);
});

test("sensor dropout holds the prior observation", () => {
  const sensor = new SensorModel({ dropoutProbability: 1, seed: 2 });
  const first = sensor.sample(state(0, 0.1));
  const held = sensor.sample(state(0.1, 0.3));
  assert.equal(held.pitch, first.pitch);
  assert.equal(held.dropout, true);
});

test("sensor quantization and seeded jitter are deterministic", () => {
  const options = { quantization: 0.05, timestampJitter: 0.02, seed: 99 };
  const left = new SensorModel(options);
  const right = new SensorModel(options);
  const leftRows = [], rightRows = [];
  for (let index = 0; index < 10; index += 1) {
    leftRows.push(left.sample(state(index * 0.02, index * 0.013)));
    rightRows.push(right.sample(state(index * 0.02, index * 0.013)));
  }
  assert.deepEqual(leftRows, rightRows);
  assert.equal(leftRows.at(-1).pitch / 0.05, Math.round(leftRows.at(-1).pitch / 0.05));
});
