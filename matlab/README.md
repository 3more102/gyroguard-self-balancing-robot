# MATLAB / Octave analysis

## Run

From the repository root in MATLAB:

    addpath('matlab');
    results = gyroguard_stage2_analysis;

To run without figures:

    results = gyroguard_stage2_analysis(false);

The script also runs in GNU Octave with:

    octave --quiet --eval "addpath('matlab'); gyroguard_stage2_analysis(false);"

No Control System Toolbox is required. The analysis uses base functions: ode45, eig, rank, odeset and plotting.

## Files

- gyroguard_parameters.m — single source for the model's physical parameters.
- gyroguard_stage2_analysis.m — nonlinear model, analytical linearization, finite-difference consistency checks, open-loop poles, controllability and response plots.

## Before using the results

The supplied parameters are illustrative placeholders. Replace them with the measured values in docs/MEASUREMENT_WORKSHEET.md. The script deliberately prints a warning while the placeholder flag is true.

Do not use placeholder simulation values as measured performance, final controller gains or proof of physical stability. The model must be compared with measured robot response before controller design is finalized.
