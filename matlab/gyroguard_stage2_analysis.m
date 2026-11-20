function results = gyroguard_stage2_analysis(makePlots)
%GYROGUARD_STAGE2_ANALYSIS Analyse the two-wheel balancing robot plant.
%
% Run from the repository root:
%   addpath('matlab');
%   results = gyroguard_stage2_analysis;
%
% Pass false to suppress figures:
%   results = gyroguard_stage2_analysis(false);
%
% No Control System Toolbox is required. The script uses base MATLAB
% functions, validates the linearization numerically, reports poles and
% controllability, and compares nonlinear and linear open-loop responses.

if nargin < 1
    makePlots = true;
end

p = gyroguard_parameters();
validateParameters(p);

% Sign sanity: positive wheel torque must drive the axle forward and apply
% the equal-and-opposite pitch torque to the body. Back-EMF must oppose
% positive wheel speed relative to the body.
positiveTestVoltage = min(1.0e-3, p.motorVoltageLimit_V);
positiveTorqueForce = motorGeneralizedForce(positiveTestVoltage, 0.0, 0.0, p);
assert(positiveTorqueForce(1) > 0.0 && positiveTorqueForce(2) < 0.0, ...
    'Positive wheel torque must act forward on the axle and oppositely on the body.');
backEmfForce = motorGeneralizedForce(0.0, 0.1, 0.0, p);
assert(backEmfForce(1) < 0.0 && backEmfForce(2) > 0.0, ...
    'Back-EMF must oppose positive wheel motion relative to the body.');
signSanityPassed = true;

[massEquivalent, bodyAxleInertia, motorForceGain, motorBackEmfDamping, ...
    effectiveDamping, rotorCrossInertia, determinant] = derivedParameters(p);
uprightCoupling = p.bodyMass_kg * p.bodyComHeight_m + rotorCrossInertia;
effectivePitchDamping = p.pitchDamping_NmsPerRad + ...
    p.wheelRadius_m^2 * motorBackEmfDamping;

[A, B] = linearizedModel(p);

% Independently verify A and B using central finite differences of the
% nonlinear state derivative at the upright equilibrium.
Afd = zeros(4, 4);
stateSteps = [1.0e-5; 1.0e-5; 1.0e-6; 1.0e-5];
for k = 1:4
    perturbation = zeros(4, 1);
    perturbation(k) = stateSteps(k);
    fPlus = nonlinearDynamics(0.0, perturbation, 0.0, p);
    fMinus = nonlinearDynamics(0.0, -perturbation, 0.0, p);
    Afd(:, k) = (fPlus - fMinus) / (2.0 * stateSteps(k));
end

voltageStep = 1.0e-5;
Bfd = (nonlinearDynamics(0.0, zeros(4, 1), voltageStep, p) - ...
       nonlinearDynamics(0.0, zeros(4, 1), -voltageStep, p)) / ...
       (2.0 * voltageStep);

linearizationErrorA = max(abs(A(:) - Afd(:)));
linearizationErrorB = max(abs(B(:) - Bfd(:)));
assert(linearizationErrorA < 1.0e-6, ...
    'Analytical A matrix does not match the nonlinear-model Jacobian.');
assert(linearizationErrorB < 1.0e-6, ...
    'Analytical B matrix does not match the nonlinear-model Jacobian.');

openLoopPoles = eig(A);
controllabilityMatrix = [B, A * B, A^2 * B, A^3 * B];
controllabilityRank = rank(controllabilityMatrix);

fprintf('\nGYROGUARD STAGE 2 MODEL ANALYSIS\n');
fprintf('Parameter set: %s\n', p.parameterSet);
if p.isIllustrative
    fprintf(['WARNING: numerical parameters are placeholders. Replace them ' ...
             'before reporting physical performance or designing final gains.\n']);
end
fprintf('Equivalent wheel/rotor translational mass M: %.6f kg\n', massEquivalent);
fprintf('Body and motor-rotor pitch inertia J: %.6f kg m^2\n', bodyAxleInertia);
fprintf('Upright body/rotor coupling h0: %.6f kg m\n', uprightCoupling);
fprintf('Motor force gain Ku: %.6f N/V\n', motorForceGain);
fprintf('Motor back-EMF force coefficient Be: %.6f N s/m\n', motorBackEmfDamping);
fprintf('Effective translational damping b: %.6f N s/m\n', effectiveDamping);
fprintf('Effective pitch damping c_eff: %.6f N m s/rad\n', effectivePitchDamping);
fprintf('Motor-rotor cross-inertia contribution: %.6f kg m\n', rotorCrossInertia);
fprintf('Mass-matrix determinant at upright Delta: %.9f\n', determinant);
fprintf('Maximum finite-difference error in A: %.3e\n', linearizationErrorA);
fprintf('Maximum finite-difference error in B: %.3e\n', linearizationErrorB);
fprintf('Motor generalized-force sign sanity: passed\n');
fprintf('Open-loop poles (rad/s):\n');
disp(openLoopPoles);
fprintf('Controllability rank: %d / 4\n', controllabilityRank);

if any(real(openLoopPoles) > 0.0)
    fprintf('Upright open-loop equilibrium is unstable, as expected.\n');
else
    warning('No positive-real-part pole was found; review model signs and parameters.');
end
if controllabilityRank < 4
    warning('The linearized plant is not fully controllable with the selected parameters.');
end

% Start with a 3-degree forward pitch and zero motor voltage. Stop the
% nonlinear integration when the body reaches the 20-degree test boundary.
initialState = [0.0; 0.0; 3.0 * pi / 180.0; 0.0];
tiltLimitRad = 20.0 * pi / 180.0;
nonlinearOptions = odeset('RelTol', 1.0e-8, 'AbsTol', 1.0e-9, ...
    'MaxStep', 0.005, ...
    'Events', @(t, x) tiltLimitEvent(t, x, tiltLimitRad));
[timeNonlinear, stateNonlinear] = ode45( ...
    @(t, x) nonlinearDynamics(t, x, 0.0, p), ...
    [0.0, 2.0], initialState, nonlinearOptions);

linearOptions = odeset('RelTol', 1.0e-8, 'AbsTol', 1.0e-9, ...
    'MaxStep', 0.005);
[timeLinear, stateLinear] = ode45( ...
    @(t, x) A * x, [0.0, timeNonlinear(end)], ...
    initialState, linearOptions);

if abs(stateNonlinear(end, 3)) >= 0.999 * tiltLimitRad
    fprintf('Nonlinear open-loop response reached 20 degrees at %.4f s.\n', ...
        timeNonlinear(end));
else
    fprintf('Nonlinear response did not reach 20 degrees in the 2 s horizon.\n');
end

results = struct();
results.parameters = p;
results.A = A;
results.B = B;
results.openLoopPoles = openLoopPoles;
results.controllabilityRank = controllabilityRank;
results.linearizationErrorA = linearizationErrorA;
results.linearizationErrorB = linearizationErrorB;
results.signSanityPassed = signSanityPassed;
results.massEquivalent = massEquivalent;
results.bodyAxleInertia = bodyAxleInertia;
results.uprightCoupling = uprightCoupling;
results.motorForceGain = motorForceGain;
results.motorBackEmfDamping = motorBackEmfDamping;
results.effectiveDamping = effectiveDamping;
results.effectivePitchDamping = effectivePitchDamping;
results.rotorCrossInertia = rotorCrossInertia;
results.timeNonlinear = timeNonlinear;
results.stateNonlinear = stateNonlinear;
results.timeLinear = timeLinear;
results.stateLinear = stateLinear;

if makePlots
    figure('Name', 'GyroGuard Stage 2 — open-loop model');
    subplot(2, 1, 1);
    plot(timeNonlinear, stateNonlinear(:, 3) * 180.0 / pi, ...
        'LineWidth', 1.5);
    hold on;
    plot(timeLinear, stateLinear(:, 3) * 180.0 / pi, '--', ...
        'LineWidth', 1.2);
    plot([0.0, timeNonlinear(end)], [20.0, 20.0], ':');
    plot([0.0, timeNonlinear(end)], [-20.0, -20.0], ':');
    grid on;
    xlabel('Time (s)');
    ylabel('Pitch (deg)');
    legend('Nonlinear model', 'Linearized model', '20 deg boundary', ...
        'Location', 'best');
    title('Open-loop response from a 3-degree forward pitch');

    subplot(2, 1, 2);
    plot(timeNonlinear, stateNonlinear(:, 1), 'LineWidth', 1.5);
    hold on;
    plot(timeLinear, stateLinear(:, 1), '--', 'LineWidth', 1.2);
    grid on;
    xlabel('Time (s)');
    ylabel('Axle position (m)');
    legend('Nonlinear model', 'Linearized model', 'Location', 'best');
end
end

function validateParameters(p)
positiveValues = [p.gravity_mps2, p.bodyMass_kg, p.bodyComHeight_m, ...
    p.bodyPitchInertia_kgm2, p.wheelMass_kg, p.wheelRadius_m, ...
    p.motorGearRatio, p.motorResistance_ohm, ...
    p.motorTorqueConstant_NmPerA, p.motorBackEmfConstant_VsPerRad, ...
    p.motorVoltageLimit_V];
assert(all(isfinite(positiveValues)) && all(positiveValues > 0.0), ...
    'Mass, geometry, motor and gravity parameters must be finite and positive.');

nonnegativeValues = [p.wheelInertia_kgm2, p.motorRotorInertia_kgm2, ...
    p.translationalDamping_NsPerM, p.pitchDamping_NmsPerRad];
assert(all(isfinite(nonnegativeValues)) && all(nonnegativeValues >= 0.0), ...
    'Inertia and damping parameters must be finite and nonnegative.');

assert(isfinite(p.motorEfficiency) && ...
    p.motorEfficiency > 0.0 && p.motorEfficiency <= 1.0, ...
    'Motor efficiency must be in the interval (0, 1].');
end

function [M, J, Ku, Be, b, rotorCrossInertia, Delta] = derivedParameters(p)
rotorPitchInertia = 2.0 * p.motorRotorInertia_kgm2 * ...
    (1.0 - p.motorGearRatio)^2;
rotorCrossInertia = 2.0 * p.motorRotorInertia_kgm2 * ...
    p.motorGearRatio * (1.0 - p.motorGearRatio) / p.wheelRadius_m;

M = 2.0 * p.wheelMass_kg + ...
    2.0 * p.wheelInertia_kgm2 / p.wheelRadius_m^2 + ...
    2.0 * p.motorRotorInertia_kgm2 * p.motorGearRatio^2 / p.wheelRadius_m^2;
J = p.bodyPitchInertia_kgm2 + ...
    p.bodyMass_kg * p.bodyComHeight_m^2 + rotorPitchInertia;

Ku = 2.0 * p.motorEfficiency * p.motorGearRatio * ...
    p.motorTorqueConstant_NmPerA / ...
    (p.motorResistance_ohm * p.wheelRadius_m);
Be = 2.0 * p.motorEfficiency * p.motorGearRatio^2 * ...
    p.motorTorqueConstant_NmPerA * p.motorBackEmfConstant_VsPerRad / ...
    (p.motorResistance_ohm * p.wheelRadius_m^2);
b = p.translationalDamping_NsPerM + Be;

h0 = p.bodyMass_kg * p.bodyComHeight_m + rotorCrossInertia;
Delta = (M + p.bodyMass_kg) * J - h0^2;
assert(isfinite(Delta) && Delta > 0.0, ...
    'The coupled mass matrix must be positive and nonsingular at upright.');
end

function [A, B] = linearizedModel(p)
[M, J, Ku, Be, b, rotorCrossInertia, Delta] = derivedParameters(p);
m = p.bodyMass_kg;
l = p.bodyComHeight_m;
r = p.wheelRadius_m;
g = p.gravity_mps2;
cEff = p.pitchDamping_NmsPerRad + r^2 * Be;
h0 = m * l + rotorCrossInertia;
a = M + m;

A = zeros(4, 4);
A(1, 2) = 1.0;
A(2, 2) = (-J * b - h0 * r * Be) / Delta;
A(2, 3) = -h0 * m * g * l / Delta;
A(2, 4) = (J * r * Be + h0 * cEff) / Delta;
A(3, 4) = 1.0;
A(4, 2) = (h0 * b + a * r * Be) / Delta;
A(4, 3) = a * m * g * l / Delta;
A(4, 4) = (-h0 * r * Be - a * cEff) / Delta;

B = zeros(4, 1);
B(2) = (J + h0 * r) * Ku / Delta;
B(4) = -(h0 + a * r) * Ku / Delta;
end

function dx = nonlinearDynamics(~, x, motorVoltage, p)
[M, J, ~, ~, ~, rotorCrossInertia, ~] = derivedParameters(p);
motorVoltage = min(max(motorVoltage, -p.motorVoltageLimit_V), ...
    p.motorVoltageLimit_V);

m = p.bodyMass_kg;
l = p.bodyComHeight_m;
r = p.wheelRadius_m;
g = p.gravity_mps2;
v = x(2);
theta = x(3);
thetaRate = x(4);

motorQ = motorGeneralizedForce(motorVoltage, v, thetaRate, p);
coupling = m * l * cos(theta) + rotorCrossInertia;
massMatrix = [M + m, coupling; ...
              coupling, J];

rhs = [motorQ(1) - p.translationalDamping_NsPerM * v + ...
       m * l * thetaRate^2 * sin(theta); ...
       m * g * l * sin(theta) - ...
       p.pitchDamping_NmsPerRad * thetaRate + motorQ(2)];

accelerations = massMatrix \ rhs;
dx = [v; accelerations(1); thetaRate; accelerations(2)];
end

function generalizedForce = motorGeneralizedForce(motorVoltage, linearVelocity, pitchRate, p)
% Return generalized motor forces for q = [axle position; body pitch].
% Motor torque acts between wheel and chassis: [tau/r; -tau].
relativeMotorSpeed = p.motorGearRatio * ...
    (linearVelocity / p.wheelRadius_m - pitchRate);
motorCurrent = (motorVoltage - ...
    p.motorBackEmfConstant_VsPerRad * relativeMotorSpeed) / ...
    p.motorResistance_ohm;
totalWheelTorque = 2.0 * p.motorEfficiency * p.motorGearRatio * ...
    p.motorTorqueConstant_NmPerA * motorCurrent;
generalizedForce = [totalWheelTorque / p.wheelRadius_m; -totalWheelTorque];
end

function [value, isterminal, direction] = tiltLimitEvent(~, x, limitRad)
value = limitRad - abs(x(3));
isterminal = 1;
direction = -1;
end
