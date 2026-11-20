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
- \(\phi=s/r\): wheel rotation angle, positive for forward rolling (rad)

At the upright equilibrium, \(s=\dot{s}=\theta=\dot{\theta}=u=0\). The controller's common-mode motor command acts on both wheels. Differential voltage / turn bias is outside this single-axis model.

The motor stators are attached to the body. Therefore the motor rotor speed relative to its stator is based on wheel rotation **relative to the body**, not just wheel speed relative to the floor.

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
| \(G\) | Gear ratio: motor speed divided by wheel speed relative to the body | dimensionless |
| \(\eta\) | Motor/gearbox mechanical efficiency | dimensionless |
| \(R\) | Armature resistance of one motor | Ω |
| \(K_t\) | Motor torque constant | N m/A |
| \(K_e\) | Motor back-EMF constant | V s/rad |
| \(b_m\) | Lumped translational mechanical damping | N s/m |
| \(c_\theta\) | Lumped pitch damping, excluding motor electrical damping | N m s/rad |
| \(g\) | Gravitational acceleration | m/s² |

The motor electrical inductance is neglected, so current is approximated as an algebraic function of applied voltage and rotor speed relative to the stator. This approximation is suitable for a first reduced-order plant model, not a detailed current-loop model.

## 4. Wheel, body and motor terms

The reflected motor-rotor inertia must account for the fact that the motor stator rotates with the body. With motor-rotor absolute angle

\[
\alpha_m = G\phi+(1-G)\theta
\]

the two identical motor rotors contribute the following terms to the mass matrix:

\[
M = 2m_w+\frac{2J_w}{r^2}+\frac{2J_mG^2}{r^2}
\]

\[
J = I_b+m_bl^2+2J_m(1-G)^2
\]

\[
h(\theta)=m_bl\cos\theta+\frac{2J_mG(1-G)}{r}
\]

Thus the motor-rotor inertia contributes to the translational inertia, pitch inertia, and off-diagonal coupling. Including it only in \(M\) would be inconsistent.

For two identical motors receiving the same voltage, the total wheel-side motor torque is

\[
\tau_m =
\frac{2\eta G K_t}{R}u
-
\frac{2\eta G^2 K_t K_e}{R}
\left(\frac{\dot{s}}{r}-\dot{\theta}\right)
\]

The relative-speed term is essential: the rotor spins relative to the body-mounted stator. Define the wheel-side force \(F_m=\tau_m/r\), giving

\[
F_m=K_u u-B_e\dot{s}+rB_e\dot{\theta}
\]

where

\[
K_u=\frac{2\eta G K_t}{Rr},
\qquad
B_e=\frac{2\eta G^2K_tK_e}{Rr^2}
\]

Motor torque acts between the wheel and body. For generalized coordinates \([s,\theta]^T\), its generalized forces are

\[
\mathbf{Q}_m =
\begin{bmatrix}
F_m\\
-\tau_m
\end{bmatrix}
\]

The second component is the equal-and-opposite reaction torque on the body. Omitting it produces a cart-pendulum approximation rather than the motor-driven wheeled-inverted-pendulum model. The analysis code checks this sign convention explicitly: positive wheel torque must produce a positive forward generalized force and a negative body-pitch generalized torque; zero-voltage back-EMF must oppose positive relative wheel motion.

For compact notation, define

\[
b=b_m+B_e,
\qquad
c_{\mathrm{eff}}=c_\theta+r^2B_e
\]

The back-EMF terms therefore appear in both generalized-force components, with the signs shown below.

## 5. Nonlinear equations

Using the upright-positive pitch convention, the coupled equations are

\[
\begin{bmatrix}
M+m_b & h(\theta) \\
h(\theta) & J
\end{bmatrix}
\begin{bmatrix}
\ddot{s}\\
\ddot{\theta}
\end{bmatrix}
=
\begin{bmatrix}
K_u u-b\dot{s}+rB_e\dot{\theta}
+m_bl\dot{\theta}^{2}\sin\theta\\
m_bgl\sin\theta-rK_u u+rB_e\dot{s}
-c_{\mathrm{eff}}\dot{\theta}
\end{bmatrix}
\]

At zero input, a positive forward pitch produces positive angular acceleration: the upright configuration is open-loop unstable. Positive motor voltage produces forward axle acceleration and a negative pitch-acceleration contribution, corresponding to driving the wheels underneath the falling body.

The MATLAB script evaluates these equations directly and integrates them with ode45.

## 6. Linearization about upright

For small pitch angles,

\[
\sin\theta \approx \theta,\qquad \cos\theta \approx 1
\]

Define the upright coupling and mass-matrix determinant

\[
h_0=m_bl+\frac{2J_mG(1-G)}{r}
\]

\[
\Delta=(M+m_b)J-h_0^2
\]

Then

\[
\dot{\mathbf{x}}=A\mathbf{x}+Bu
\]

where

\[
A=
\begin{bmatrix}
0 & 1 & 0 & 0\\
0 & \frac{-Jb-h_0rB_e}{\Delta} &
\frac{-h_0m_bgl}{\Delta} &
\frac{JrB_e+h_0c_{\mathrm{eff}}}{\Delta}\\
0 & 0 & 0 & 1\\
0 & \frac{h_0b+(M+m_b)rB_e}{\Delta} &
\frac{(M+m_b)m_bgl}{\Delta} &
\frac{-h_0rB_e-(M+m_b)c_{\mathrm{eff}}}{\Delta}
\end{bmatrix}
\]

and

\[
B=
\begin{bmatrix}
0\\
\frac{(J+h_0r)K_u}{\Delta}\\
0\\
-\frac{(h_0+(M+m_b)r)K_u}{\Delta}
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
3. Checks the motor generalized-force signs: positive wheel torque drives the axle forward and applies the opposing torque to the body; back-EMF opposes positive relative wheel motion.
4. Independently estimates the local Jacobians of the nonlinear state derivative using central finite differences and asserts agreement with \(A\) and \(B\).
5. Computes the open-loop eigenvalues.
6. Builds the controllability matrix and reports its rank.
7. Simulates a small initial forward pitch with zero motor voltage, stopping at a 20-degree pitch limit.
8. Compares nonlinear and linearized pitch responses.

The script uses base MATLAB functions (including ode45, eig and rank); it does not require the Control System Toolbox. Pass false to gyroguard_stage2_analysis(false) to run the numerical checks without opening figures.

## 8. Assumptions and limitations

- The robot is treated as a rigid body plus two identical rolling wheels and two identical body-mounted geared motors.
- Both wheels have equal common-mode drive voltage; yaw and differential steering are excluded.
- Wheels roll without slip and remain in contact with the floor.
- The body centre of mass is above the axle.
- Motor rotor inertia is included using the motor rotor's absolute angular velocity; gearbox and other rotating drivetrain inertias are not separately identified.
- The motor electrical inductance is neglected.
- Gear efficiency and motor constants are treated as constant.
- Mechanical translational and pitch damping are lumped into \(b_m\) and \(c_\theta\); motor back-EMF damping is modelled separately.
- Motor voltage is limited in the nonlinear simulation; the state-space model is a local small-signal model around zero voltage.
- The model does not include the discrete-time PID implementation, sensor filtering, computation delay or PWM quantization.

For final analysis, update the parameter file with measured values and document the measurement method. If motor or friction parameters are unavailable, identify them experimentally rather than presenting placeholder values as measured constants.
