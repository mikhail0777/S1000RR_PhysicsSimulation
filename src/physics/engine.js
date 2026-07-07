const clamp = (value, low, high) => Math.min(high, Math.max(low, value));

/** Smooth, deliberately approximate full-load curve normalized to published peak torque. */
export function fullLoadTorque(rpm, engine) {
  const x = clamp(rpm, engine.idleRpm, engine.redlineRpm);
  const points = [
    [1300, 58], [3000, 75], [5000, 88], [7000, 99], [8500, 92],
    [10000, 108], [11000, engine.peakTorqueNm], [13000, 108], [14600, 91]
  ];
  for (let i = 1; i < points.length; i += 1) {
    if (x <= points[i][0]) {
      const [x0, y0] = points[i - 1];
      const [x1, y1] = points[i];
      return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
    }
  }
  return points.at(-1)[1];
}

/** Grip is mapped nonlinearly; closed throttle produces explicit engine braking. */
export function riderRequestedTorque(throttle, rpm, engine) {
  const grip = clamp(throttle, 0, 1);
  const positive = fullLoadTorque(rpm, engine) * Math.pow(grip, 1.65);
  const braking = -engine.brakingTorque * Math.pow(1 - grip, 2) * clamp(rpm / 7000, 0.15, 1.5);
  const blend = clamp(grip / 0.08, 0, 1);
  return braking * (1 - blend) + positive * blend;
}

export function updateEngine(state, permittedTorque, dt, engine) {
  state.commandQueue.push({ time: state.time + engine.transportDelay, torque: permittedTorque });
  while (state.commandQueue.length && state.commandQueue[0].time <= state.time + 1e-12) {
    state.delayedTorque = state.commandQueue.shift().torque;
  }
  const desiredRate = (state.delayedTorque - state.engineTorque) / Math.max(engine.responseTime, 1e-4);
  const rate = clamp(desiredRate, -engine.maxTorqueRate, engine.maxTorqueRate);
  state.engineTorque += rate * dt;
  return state.engineTorque;
}

