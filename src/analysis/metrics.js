const mean = values => values.length ? values.reduce((sum, value) => sum + value, 0) / values.length : 0;

export function evaluateRun(run, tolerance = 2 * Math.PI / 180) {
  const samples = run.history;
  const airborne = samples.filter(item => item.mode === "wheelie");
  const tracking = airborne.length ? airborne : samples;
  const squaredErrors = tracking.map(item => (item.pitch - item.targetPitch) ** 2);
  const overshoot = Math.max(0, ...samples.map(item => item.pitch - item.targetPitch));
  const violations = samples.reduce((sum, item) => sum + item.violations.length, 0);
  const within = tracking.filter(item => Math.abs(item.pitch - item.targetPitch) <= tolerance).length;
  const maxPitch = Math.max(...samples.map(item => item.pitch));
  const lifted = samples.some(item => item.mode === "wheelie");
  const requestedSupport = tracking.filter(item => item.controllerStatus === "insufficient-rider-demand").length;
  let classification = "limited/partial tracking";
  if (!lifted && run.scenario.targetPitch > tolerance) classification = "physically unreachable target";
  else if (violations > 0 || samples.some(item => item.unsafe)) classification = "safety constraint failure";
  else if (within / tracking.length > 0.6) classification = "feasible tracking";
  else if (requestedSupport / tracking.length > 0.25) classification = "physically unreachable target";
  return {
    classification,
    pitchRmseDeg: Math.sqrt(mean(squaredErrors)) * 180 / Math.PI,
    maxOvershootDeg: overshoot * 180 / Math.PI,
    maxPitchDeg: maxPitch * 180 / Math.PI,
    maxPitchRateDegS: Math.max(...samples.map(item => Math.abs(item.pitchRate))) * 180 / Math.PI,
    timeWithinTolerance: within * (run.history[1]?.time - run.history[0]?.time || 0),
    interventionRmsNm: Math.sqrt(mean(samples.map(item => item.intervention ** 2))),
    saturationDuration: samples.filter(item => item.permittedTorque <= 1e-6 && item.requestedTorque > 0).length *
      (run.history[1]?.time - run.history[0]?.time || 0),
    constraintViolations: violations,
    controllerMeanComputeMs: mean(samples.map(item => item.computeMs)),
    controllerMaxComputeMs: Math.max(...samples.map(item => item.computeMs)),
    frontLifted: lifted
  };
}

