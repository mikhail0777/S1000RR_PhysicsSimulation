const ramp = (time, start, end, from, to) => {
  if (time <= start) return from;
  if (time >= end) return to;
  return from + (to - from) * (time - start) / (end - start);
};

export const scenarios = Object.freeze({
  unassisted: {
    title: "A — Unassisted acceleration",
    duration: 6,
    initialSpeed: 12,
    gear: 1,
    targetPitch: 0.22,
    controller: "none",
    throttle: time => ramp(time, 0.3, 1.2, 0.08, 0.92)
  },
  ideal: {
    title: "B — Ideal pitch stabilization",
    duration: 7,
    initialSpeed: 12,
    gear: 1,
    targetPitch: 0.20,
    controller: "pid",
    engineResponseTime: 0.005,
    throttle: time => ramp(time, 0.3, 1.0, 0.1, 0.9)
  },
  insufficient: {
    title: "C — Insufficient rider demand",
    duration: 7,
    initialSpeed: 15,
    initialPitch: 0.24,
    gear: 2,
    targetPitch: 0.24,
    controller: "pid",
    throttle: time => time < 1.4 ? 0.55 : 0.10
  },
  excessive: {
    title: "D — Excessive rider demand",
    duration: 6,
    initialSpeed: 12,
    gear: 1,
    targetPitch: 0.18,
    controller: "delay-aware",
    throttle: time => time < 0.4 ? 0.12 : 1
  },
  delay: {
    title: "E — Engine and sensor delay",
    duration: 7,
    initialSpeed: 12,
    gear: 1,
    targetPitch: 0.18,
    controller: "delay-aware",
    engineDelay: 0.10,
    sensor: { latency: 0.045 },
    throttle: time => ramp(time, 0.2, 0.9, 0.08, 0.95)
  },
  noise: {
    title: "F — Sensor noise",
    duration: 7,
    initialSpeed: 12,
    gear: 1,
    targetPitch: 0.18,
    controller: "pid",
    sensor: { pitchNoise: 0.007, rateNoise: 0.035, quantization: 0.001 },
    throttle: time => ramp(time, 0.2, 1, 0.1, 0.9)
  },
  centerOfMass: {
    title: "G — Forward / high rider position",
    duration: 7,
    initialSpeed: 12,
    gear: 1,
    targetPitch: 0.18,
    controller: "pid",
    comFromRear: 0.77,
    comHeight: 0.71,
    throttle: time => ramp(time, 0.2, 1, 0.1, 0.92)
  },
  traction: {
    title: "H — Limited traction",
    duration: 6,
    initialSpeed: 10,
    gear: 1,
    targetPitch: 0.18,
    controller: "pid",
    friction: 0.55,
    throttle: time => time < 0.4 ? 0.1 : 1
  },
  fault: {
    title: "I — Controller command interruption",
    duration: 7,
    initialSpeed: 12,
    gear: 1,
    targetPitch: 0.18,
    controller: "pid",
    faultWindow: [2.3, 2.75],
    throttle: time => ramp(time, 0.2, 0.9, 0.1, 0.95)
  }
});

export function getScenario(name) {
  const scenario = scenarios[name];
  if (!scenario) throw new Error(`Unknown scenario: ${name}`);
  return scenario;
}

