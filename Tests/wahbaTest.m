%% Test Suite for wahbaSolver
addpath("../Functions/")
% Define a known "true" rotation (90 degrees about the Z-axis)
theta = pi/2;
R_true = [cos(theta),  sin(theta), 0;
         -sin(theta),  cos(theta), 0;
          0,           0,          1];

% Tolerance for floating-point comparisons
tol = 1e-10;

%% Case 1: The Nominal Case (Perfect Orthogonal Measurements)
% 3 perfectly orthogonal vectors, equal weights, zero noise.
vIMat_1 = [1 0 0; 0 1 0; 0 0 1];
vBMat_1 = (R_true * vIMat_1')'; 
aVec_1  = [1; 1; 1];

R_calc_1 = wahbaSolver(aVec_1, vIMat_1, vBMat_1);
assert(norm(R_calc_1 - R_true) < tol, 'Case 1 Failed: Nominal');
disp('Case 1 Passed: Nominal');

%% Case 5: The Reflection Case (Left-Handed Body Frame)
% Create a standard right-handed orthogonal set for the Inertial frame
vIMat_5 = [1 0 0; 
           0 1 0; 
           0 0 1];

% Create a Body frame that is a reflection of the Inertial frame
% We do this by flipping the Z-axis, making it a left-handed system
vBMat_5 = [1  0  0; 
           0  1  0; 
           0  0 -1]; 

aVec_5  = [1; 1; 1];

% Run the solver
R_calc_5 = wahbaSolver(aVec_5, vIMat_5, vBMat_5);

% 1. Check that the determinant is strictly +1 (It is a valid rotation)
det_R = det(R_calc_5);
assert(abs(det_R - 1) < 1e-10, 'Case 5 Failed: Matrix is a reflection (det = -1), not a rotation.');

% 2. Check that the matrix is still orthogonal
assert(norm(R_calc_5'*R_calc_5 - eye(3)) < 1e-10, 'Case 5 Failed: DCM is not orthogonal.');

disp('Case 2 Passed: Reflection successfully prevented.');

%% Case 3: The Collinear/Degenerate Case
% Two vectors pointing in the exact same direction. The 3D attitude is unconstrained 
% about that vector. The solver shouldn't crash, but it won't yield R_true.
vIMat_3 = [1 0 0; 1 0 0];
vBMat_3 = (R_true * vIMat_3')';
aVec_3  = [1; 1];

R_calc_3 = wahbaSolver(aVec_3, vIMat_3, vBMat_3);
% We check that the DCM is still valid (orthogonal, det = 1), even if it's not R_true.
assert(abs(det(R_calc_3) - 1) < tol, 'Case 3 Failed: Invalid DCM determinant');
assert(norm(R_calc_3'*R_calc_3 - eye(3)) < tol, 'Case 3 Failed: DCM not orthogonal');
disp('Case 3 Passed: Collinear/Degenerate Behavior');

