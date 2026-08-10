# Validation record

Verification command:

```bash
npm run check
npm run experiment
```

The test suite verifies:

- zero throttle cannot create positive acceleration;
- tire force never exceeds `μN`;
- high first-gear drive can reach the load-transfer lift condition;
- an unpowered lifted state returns to the ground under gravity;
- increased mass responds plausibly to a common force state;
- every controller obeys the rider torque ceiling;
- the predictive controller fails closed on invalid state;
- fixed-seed noisy simulation is deterministic;
- 2 ms and 1 ms integration preserve the gross outcome;
- passive telemetry units, timestamps and channel comparison are validated.

The November 4, 2026 run passed all 18 tests. The full 45-case scenario/controller matrix plus 30 deterministic parameter-variation cases completed and exported 75 result rows. Generated result files are intentionally ignored by git so they always reflect the local code and execution environment.

The ideal near-delay-free scenario produced feasible tracking only for the equilibrium-feedforward PD/PID baseline: 2.54° airborne RMSE and 0.99° maximum overshoot. Results outside the declared safety envelope terminate immediately and are labeled `safety constraint failure`. Reduced friction did not create front lift and is labeled `physically unreachable target`.

These are simulation results, not measured BMW results.

