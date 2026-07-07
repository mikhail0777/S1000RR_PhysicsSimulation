/**
 * Baseline 2020 BMW S1000RR K67 + rider configuration.
 * Source classifications are intentionally explicit. Values marked estimate
 * are research placeholders, not measured BMW data.
 */
export const motorcycleConfig = Object.freeze({
  identity: {
    model: "2020 BMW S1000RR K67 + rider",
    disclaimer: "Research model only; not validated for real-vehicle control."
  },
  mass: 272, // kg: 197 kg road-ready motorcycle [BMW] + 75 kg rider [assumption]
  bikeMass: 197, // kg [verified: BMW 2020 technical data]
  wheelbase: 1.441, // m [verified: BMW 2020 technical data]
  comFromRear: 0.70, // m [estimate; ~49% static front load, requires scale measurement]
  comHeight: 0.66, // m [estimate; requires measurement]
  pitchInertia: 145, // kg m^2 [estimate; plausible combined bike/rider range]
  rearWheelRadius: 0.303, // m [estimate from 190/55 ZR17 geometry]
  frontWheelRadius: 0.301, // m [estimate from 120/70 ZR17 geometry]
  rearWheelInertia: 1.15, // kg m^2 [estimate]
  frontWheelInertia: 0.85, // kg m^2 [estimate]
  dragCoefficientArea: 0.38, // m^2 [estimate: CdA]
  airDensity: 1.225, // kg/m^3 [standard sea-level assumption]
  rollingResistance: 0.015, // coefficient [assumption]
  pitchDamping: 18, // N m s/rad [numerical/physical lumped estimate]
  restitution: 0.08, // touchdown angular restitution [assumption]
  gravity: 9.80665,
  tire: {
    friction: 1.15,
    longitudinalStiffness: 18000, // N per unit slip [estimate]
    relaxationTime: 0.035,
    maxSlip: 1.5
  },
  drivetrain: {
    primaryRatio: 1.652,
    finalDriveRatio: 2.647,
    gearRatios: [2.647, 2.091, 1.727, 1.476, 1.304, 1.167],
    efficiency: 0.94
  },
  engine: {
    idleRpm: 1300,
    redlineRpm: 14600,
    peakTorqueNm: 113, // representative published maximum; curve is estimated
    responseTime: 0.09, // s [illustrative, not measured BMW ECU data]
    transportDelay: 0,
    maxTorqueRate: 1800, // N m/s [illustrative]
    brakingTorque: 28 // N m maximum [illustrative]
  },
  safety: {
    warningPitch: 0.82, // rad (47 deg), model-envelope warning
    maxPitch: 1.05, // rad (60 deg), declared unsafe simulation boundary
    maxPitchRate: 4.5
  }
});

export const parameterProvenance = Object.freeze({
  verified: ["bikeMass", "wheelbase", "engine.peakTorqueNm"],
  estimated: [
    "comFromRear", "comHeight", "pitchInertia", "wheel inertias",
    "torque curve", "aerodynamic area", "tire stiffness"
  ],
  assumptions: ["rider mass", "rolling resistance", "pitch damping", "touchdown restitution"],
  needsMeasurement: [
    "combined center of mass", "pitch inertia", "wheel inertias", "on-bike torque curve",
    "engine/ECU latency", "drivetrain losses", "tire force-slip curve"
  ]
});

