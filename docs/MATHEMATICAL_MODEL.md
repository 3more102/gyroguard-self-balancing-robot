# GyroGuard Stage 2 — Mathematical Model

## 1. Model scope

This model describes the robot's **longitudinal balancing dynamics** in the sagittal plane. It is a reduced-order model for the two-wheel robot: the left and right motors receive the same common-mode voltage, and the robot is assumed to roll without slipping.

It is intended to support Stage 2 analysis and to provide a plant model for the later digital-controller design. It does not model yaw motion, obstacle-scanning servo motion, ultrasonic sensing, wheel slip, gearbox backlash, motor dead zone, battery sag, or current limiting.

**Parameter status:** the values currently supplied in matlab/gyroguard_parameters.m are illustrative placeholders. No numerical simulation result from those values is a measured performance claim for the assembled robot. Replace them with measured hardware parameters before using the model for final controller design or reporting.

## 2. Coordinates and sign convention

The state is

\[
\mathbf{x} =
\begin{bmatrix}
s & \dot{s} & \theta & \dot{\theta}
\end{bmatrix}^{T}
\]

where:

- \(s\): horizontal axle position, positive forward (m)
- \(\dot{s}\): forward axle velocity (m/s)
- \(\theta\): body pitch from upright, positive when the top of the robot leans forward (rad)
- \(\dot{\theta}\): pitch rate (rad/s)
- \(u\): average terminal voltage applied to each motor, positive for forward wheel torque (V)

At the upright equilibrium, \(s=\dot{s}=\theta=\dot{\theta}=u=0\). The controller's common-mode motor command acts on both wheels. Differential voltage / turn bias is outside this single-axis model.

## 3. Physical parameters

| Symbol | Meaning | SI unit |
|---|---|---|
| \(m_b\) | Mass of the body above the axle, including chassis, battery, electronics and motor stators; excludes the two wheels | kg |
| \(l\) | Vertical distance from axle to body centre of mass | m |
| \(I_b\) | Body pitch inertia about its centre of mass | kg m² |
| \(m_w\) | Mass of one wheel | kg |
| \(J_w\) | Rotational inertia of one wheel about its axle | kg m² |
| \(J_m\) | Rotor inertia of one motor, referred to its motor shaft | kg m² |
| \(r\) | Effective rolling radius of each wheel | m |
| \(G\) | Gear ratio: motor angular speed divided by wheel angular speed | dimensionless |
| \(\eta\) | Motor/gearbox mechanical efficiency | dimensionless |
| \(R\) | Armature resistance of one motor | Ω |
| \(K_t\) | Motor torque constant | N m/A |
| \(K_e\) | Motor back-EMF constant | V s/rad |
| \(b_m\) | Lumped translational mechanical damping | N s/m |
| \(c_\theta\) | Lumped pitch damping | N m s/rad |
| \(g\) | Gravitational acceleration | m/s² |

The motor electrical inductance is neglected, so current is approximated as an algebraic function of applied voltage and back-EMF. This approximation is suitable for a first reduced-order plant model, not a detailed current-loop model.

## 4. Equivalent wheel and motor terms

The translational equivalent mass of the two wheels and reflected motor rotors is

\[
M = 2m_w + \frac{2J_w}{r^2} + \frac{2J_mG^2}{r^2}
\]

The body inertia about the axle is

\[
J = I_b + m_b l^2
\]

For two identical motors receiving the same voltage, the approximate total drive force is

\[
F_m = K_u u - B_e\dot{s}
\]

with

\[
K_u = \frac{2\eta G K_t}{Rr},
\qquad
B_e = \frac{2\eta G^2K_tK_e}{Rr^2}
\]

The effective translational damping is

\[
b = b_m + B_e
\]

The motor back-EMF term is therefore included once in \(b\); it must not be added again as a separate force in the final equations.

## 5. Nonlinear equations

Using the upright-positive pitch convention, the coupled equations are

\[
\begin{bmatrix}
M+m_b & m_b l\cos\theta \\
m_b l\cos\theta & J
\end{bmatrix}
\begin{bmatrix}
\ddot{s}\\
\ddot{\theta}
\end{bmatrix}
=
\begin{bmatrix}
K_u u-b\dot{s}+m_b l\dot{\theta}^{2}\sin\theta\\
m_bgl\sin\theta-c_\theta\dot{\theta}
\end{bmatrix}
\]

At zero input, a positive forward pitch produces positive angular acceleration: the upright configuration is open-loop unstable. Positive motor voltage produces forward axle acceleration and a negative pitch-acceleration contribution, corresponding to driving the wheels underneath the falling body.

The MATLAB script evaluates these equations directly and integrates them with ode45.

## 6. Linearization about upright

For small pitch angles,

\[
\sin\theta \approx \theta,\qquad \cos\theta \approx 1
\]

Define

\[
\Delta = (M+m_b)J-(m_b l)^2
\]

Then

\[
\dot{\mathbf{x}} = A\mathbf{x}+Bu
\]

where

\[
A =
\begin{bmatrix}
0 & 1 & 0 & 0\\
0 & -Jb/\Delta & -m_b^2gl^2/\Delta & m_blc_\theta/\Delta\\
0 & 0 & 0 & 1\\
0 & m_lb/\Delta & (M+m_b)m_bgl/\Delta & -(M+m_b)c_\theta/\Delta
\end{bmatrix}
\]

and

\[
B =
\begin{bmatrix}
0\\
JK_u/\Delta\\
0\\
-m_blK_u/\Delta
\end{bmatrix}
\]

The output can be selected as position, velocity, pitch and pitch rate. For the present analysis the script retains all four states.

## 7. MATLAB/Octave analysis and checks

Run from the repository root:

    addpath('matlab');
    results = gyroguard_stage2_analysis;

The script:

1. Validates that the physical parameters are finite and within their admissible ranges.
2. Builds the nonlinear equations and the linearized \(A\) and \(B\) matrices.
3. Independently estimates the local Jacobians of the nonlinear equations using central finite differences and asserts agreement with \(A\) and \(B\).
4. Computes the open-loop eigenvalues.
5. Builds the controllability matrix and reports its rank.
6. Simulates a small initial forward pitch with zero motor voltage, stopping at a 20-degree pitch limit.
7. Compares nonlinear and linearized pitch responses.

The script uses base MATLAB functions (including ode45, eig and rank); it does not require the Control System Toolbox. Pass false to gyroguard_stage2_analysis(false) to run the numerical checks without opening figures.

## 8. Assumptions and limitations

- The robot is treated as a rigid body plus two identical rolling wheels.
- Both wheels have equal common-mode drive voltage; yaw and differential steering are excluded.
- Wheels roll without slip and remain in contact with the floor.
- The body centre of mass is above the axle.
- The motor electrical inductance is neglected.
- Gear efficiency and motor constants are treated as constant.
- Mechanical damping is lumped into \(b_m\) and \(c_\theta\).
- Motor voltage is limited in the nonlinear simulation; the state-space model is a local small-signal model around zero voltage.
- The model does not include the discrete-time PID implementation, sensor filtering, computation delay or PWM quantization.

For final analysis, update the parameter file with measured values and document the measurement method. If motor or friction parameters are unavailable, identify them experimentally rather than presenting placeholder values as measured constants.
