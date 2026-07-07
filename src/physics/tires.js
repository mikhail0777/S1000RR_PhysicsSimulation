const clamp = (value, low, high) => Math.min(high, Math.max(low, value));

export function longitudinalSlip(omega, radius, speed) {
  return clamp((omega * radius - speed) / Math.max(Math.abs(speed), 2), -2, 2);
}

/** Saturating brush-like longitudinal law; |Fx| cannot exceed mu*N. */
export function tireForce(slip, normalLoad, tire) {
  const limit = Math.max(0, tire.friction * normalLoad);
  if (limit === 0) return 0;
  return limit * Math.tanh(tire.longitudinalStiffness * slip / limit);
}

