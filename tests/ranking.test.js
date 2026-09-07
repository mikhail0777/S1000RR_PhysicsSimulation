import test from "node:test";
import assert from "node:assert/strict";
import { rankControllers } from "../src/analysis/ranking.js";

const row = (scenario, controller, classification, pitchRmseDeg, maxOvershootDeg = 0) => ({
  scenario, controller, classification, pitchRmseDeg, maxOvershootDeg,
  interventionRmsNm: 5, controllerMeanComputeMs: 0.1
});

test("ranking excludes scenarios unreachable for every controller", () => {
  const result = rankControllers([
    row("low-friction", "pid", "physically unreachable target", 10),
    row("low-friction", "none", "physically unreachable target", 10),
    row("ideal", "pid", "feasible tracking", 2),
    row("ideal", "none", "safety constraint failure", 15, 25)
  ]);
  assert.deepEqual(result.excludedScenarios, ["low-friction"]);
  assert.equal(result.ranking[0].controller, "pid");
});

test("safety failures dominate cosmetic tracking score", () => {
  const result = rankControllers([
    row("case", "safe", "limited/partial tracking", 8, 3),
    row("case", "unsafe", "safety constraint failure", 1, 1)
  ]);
  assert.equal(result.ranking[0].controller, "safe");
});
