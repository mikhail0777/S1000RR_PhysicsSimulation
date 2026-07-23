import { mkdir, writeFile } from "node:fs/promises";
import { scenarios } from "../src/simulation/scenarios.js";
import { runSimulation } from "../src/simulation/runner.js";
import { evaluateRun } from "../src/analysis/metrics.js";

const controllers = ["none", "threshold", "pid", "delay-aware", "nmpc"];
const selected = process.argv[2] && scenarios[process.argv[2]] ? { [process.argv[2]]: scenarios[process.argv[2]] } : scenarios;
const rows = [];
for (const [scenarioName, scenario] of Object.entries(selected)) {
  for (const controller of controllers) {
    const run = runSimulation(scenario, { controller, seed: 2020 });
    rows.push({ scenario: scenarioName, controller, ...evaluateRun(run) });
  }
}

// Reproducible parameter sensitivity around the ideal case.
for (let sample = 0; sample < 30; sample += 1) {
  const seed = 7700 + sample;
  const configRun = structuredClone((await import("../config/motorcycle.js")).motorcycleConfig);
  const phase = (sample * 2.399963229728653) % (2 * Math.PI);
  configRun.mass *= 0.88 + 0.24 * (sample / 29);
  configRun.comHeight *= 0.9 + 0.2 * (0.5 + 0.5 * Math.sin(phase));
  configRun.pitchInertia *= 0.82 + 0.36 * (0.5 + 0.5 * Math.cos(phase));
  configRun.tire.friction *= 0.75 + 0.45 * (0.5 + 0.5 * Math.sin(phase * 1.7));
  const run = runSimulation(scenarios.ideal, { controller: "delay-aware", seed, config: configRun });
  rows.push({ scenario: "monte-carlo", controller: "delay-aware", seed, ...evaluateRun(run) });
}

await mkdir("results", { recursive: true });
await writeFile("results/comparison.json", JSON.stringify({ generatedAt: new Date().toISOString(), rows }, null, 2));
const keys = [...new Set(rows.flatMap(row => Object.keys(row)))];
await writeFile("results/comparison.csv", [keys.join(","), ...rows.map(row => keys.map(key => JSON.stringify(row[key] ?? "")).join(","))].join("\n"));
console.table(rows.filter(row => row.scenario !== "monte-carlo").map(row => ({
  scenario: row.scenario, controller: row.controller, outcome: row.classification,
  rmse_deg: row.pitchRmseDeg.toFixed(2), overshoot_deg: row.maxOvershootDeg.toFixed(2),
  intervention_nm: row.interventionRmsNm.toFixed(1)
})));
console.log(`\nWrote ${rows.length} runs to results/comparison.{json,csv}`);

