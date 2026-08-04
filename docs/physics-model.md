# Physics model and governing equations

## Coordinates and states

The inertial `x` axis points forward and `z` points upward. Body pitch `θ` is zero on level ground and positive nose-up. The integrated states are longitudinal position and speed, pitch and pitch rate, front/rear wheel speed, actual engine torque, delayed command state, and relaxed rear-slip state. SI units are used internally.

The rear contact is the wheelie-mode reference. With `b` the rear-contact-to-COM distance along the body and `h` COM height above the axle line,

```text
x_COM(θ) = b cos θ − h sin θ
z_COM(θ) = r_r + b sin θ + h cos θ
front clearance = max(0, L sin θ)
```

## Ground mode

Longitudinal motion is

```text
m a = F_x − ½ ρ C_d A v|v| − C_rr m g sign(v)
```

Quasi-static load transfer gives

```text
N_f = [m g b − m a h] / L
N_r = m g − N_f
```

The model changes to wheelie mode when the unconstrained `N_f ≤ 0` while accelerating. Ground mode constrains `θ = 0`; it does not rotate a sprite from throttle input.

## Wheelie mode

The pitch equation is a reduced moment balance about the rear support with longitudinal inertial loading:

```text
I_θ θ̈ = m a z_COM(θ) − m g x_COM(θ) − c_θ θ̇
```

Gravity therefore restores the front whenever forward acceleration is below

```text
a_hold(θ) = g x_COM(θ) / z_COM(θ)
```

The required holding torque is not zero and varies with pitch, gear, geometry and losses. If rider-requested torque falls below that requirement, a reduction-only controller cannot maintain the reference. Touchdown occurs at `θ ≤ 0` with downward pitch rate; pitch is projected to zero and a small angular restitution is applied.

## Tire and wheel

```text
κ = (r_r ω_r − v) / max(|v|, 2 m/s)
F_x = μ N_r tanh(C_κ κ / μ N_r)
I_w ω̇_r = T_w − F_x r_r − T_loss
```

The hyperbolic tire law is smooth for numerical integration and guarantees `|F_x| ≤ μN_r`. It is a longitudinal research approximation, not a Pacejka fit to a measured tire.

## Engine and drivetrain

Grip demand is mapped through a nonlinear exponent and an estimated piecewise torque curve. Closed grip blends into explicit negative engine torque. A command queue implements transport delay; actual torque follows a first-order lag and rate limit. Wheel torque is

```text
T_w = T_engine · i_primary · i_gear · i_final · η
```

The plant finally clamps positive permitted torque to the rider request. This defense in depth means a faulty controller cannot grant itself extra positive authority.

## Numerical method

The plant uses fixed-step semi-implicit Euler integration at 2 ms by default. Controllers and sensors update independently at 10 ms. The convergence test compares 2 ms and 1 ms outcomes. Safety-boundary termination prevents extrapolation after 60° pitch or 4.5 rad/s pitch rate.

## Known omissions

Suspension travel, chain-pull geometry, tire vertical dynamics, road slope, rider articulation, chassis flex, clutch slip, detailed engine combustion, aerodynamic pitch moment and coupled wheel/rigid-body constraints are not modeled. Front normal load is quasi-static on the ground. These omissions make the simulator suitable for hypothesis exploration and software/control experiments—not vehicle certification.

