const mean = values => values.length ? values.reduce((sum, value) => sum + value, 0) / values.length : 0;

/** Aggregate controllers while excluding scenarios unreachable for every controller. */
export function rankControllers(rows) {
  const scenarioNames = [...new Set(rows.filter(row => row.scenario !== "monte-carlo").map(row => row.scenario))];
  const excludedScenarios = scenarioNames.filter(name => {
    const scenarioRows = rows.filter(row => row.scenario === name);
    return scenarioRows.length > 0 && scenarioRows.every(row => row.classification === "physically unreachable target");
  });
  const included = rows.filter(row => row.scenario !== "monte-carlo" && !excludedScenarios.includes(row.scenario));
  const controllers = [...new Set(included.map(row => row.controller))];
  const ranking = controllers.map(controller => {
    const controllerRows = included.filter(row => row.controller === controller);
    const count = classification => controllerRows.filter(row => row.classification === classification).length;
    const safetyFailures = count("safety constraint failure");
    const limited = count("limited/partial tracking");
    const feasible = count("feasible tracking");
    const unreachable = count("physically unreachable target");
    const meanRmseDeg = mean(controllerRows.map(row => Math.min(60, row.pitchRmseDeg)));
    const meanOvershootDeg = mean(controllerRows.map(row => Math.min(60, row.maxOvershootDeg)));
    const meanInterventionNm = mean(controllerRows.map(row => row.interventionRmsNm));
    const meanComputeMs = mean(controllerRows.map(row => row.controllerMeanComputeMs));
    const score = safetyFailures * 100 + limited * 10 + unreachable * 6 - feasible * 5 +
      meanRmseDeg + 0.5 * meanOvershootDeg + 0.01 * meanInterventionNm;
    return { controller, score, feasible, limited, unreachable, safetyFailures,
      meanRmseDeg, meanOvershootDeg, meanInterventionNm, meanComputeMs };
  }).sort((left, right) => left.score - right.score)
    .map((row, index) => ({ rank: index + 1, ...row }));
  return { ranking, excludedScenarios };
}

