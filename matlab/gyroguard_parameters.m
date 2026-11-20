function p = gyroguard_parameters()
%GYROGUARD_PARAMETERS Physical parameters for the Stage 2 robot model.
%
% IMPORTANT:
% These defaults are illustrative placeholders, not measurements of the
% user's assembled robot. Replace them using docs/MEASUREMENT_WORKSHEET.md.
%
% Mass convention:
%   bodyMass_kg includes the chassis, battery, electronics, motor stators,
%   brackets and sensor assembly above the axle; it excludes both wheels.
% Inertia convention:
%   bodyPitchInertia_kgm2 is about the body centre of mass.
%   motorRotorInertia_kgm2 is for one motor at its motor shaft.
% Gear ratio:
%   motorGearRatio is motor-shaft speed divided by wheel speed.

p.parameterSet = 'ILLUSTRATIVE PLACEHOLDERS - replace with measured hardware data';
p.isIllustrative = true;

p.gravity_mps2 = 9.81;

% Mechanical parameters (illustrative only).
p.bodyMass_kg = 0.45;
p.bodyComHeight_m = 0.09;
p.bodyPitchInertia_kgm2 = 3.0e-3;

p.wheelMass_kg = 0.035;
p.wheelRadius_m = 0.0325;
p.wheelInertia_kgm2 = 0.5 * p.wheelMass_kg * p.wheelRadius_m^2;

% Two identical geared DC motors (illustrative only).
p.motorRotorInertia_kgm2 = 1.0e-7;
p.motorGearRatio = 48.0;
p.motorEfficiency = 0.75;
p.motorResistance_ohm = 4.0;
p.motorTorqueConstant_NmPerA = 0.006;
p.motorBackEmfConstant_VsPerRad = 0.006;
p.motorVoltageLimit_V = 6.0;

% Lumped mechanical losses (illustrative only).
p.translationalDamping_NsPerM = 0.05;
p.pitchDamping_NmsPerRad = 0.002;
end
