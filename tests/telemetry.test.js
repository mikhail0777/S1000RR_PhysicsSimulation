import test from "node:test";
import assert from "node:assert/strict";
import { compareTelemetry, parseTelemetryCsv } from "../src/telemetry/import.js";

test("telemetry import validates units and normalizes degrees and percentages", () => {
  const data = parseTelemetryCsv("time_s,engine_rpm,throttle_pct,pitch_deg\n0,4000,50,10\n0.1,4100,55,12\n");
  assert.equal(data.rows[0].throttle, 0.5);
  assert.ok(Math.abs(data.rows[0].pitch - 10 * Math.PI / 180) < 1e-12);
  assert.deepEqual(data.missingChannels, ["rearSpeed", "frontSpeed"]);
});

test("telemetry import rejects duplicate timestamps", () => {
  assert.throws(() => parseTelemetryCsv("time_s,engine_rpm\n0,4000\n0,4100"), /strictly/);
});

test("telemetry comparison reports channel RMSE", () => {
  const measured = parseTelemetryCsv("time_s,engine_rpm\n0,4000\n1,5000");
  const errors = compareTelemetry(measured, [{ time: 0, engineRpm: 3900 }, { time: 1, engineRpm: 5100 }]);
  assert.equal(errors.rpm.rmse, 100);
});

