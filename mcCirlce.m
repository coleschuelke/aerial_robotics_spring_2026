clear all;
close all;
clc;

addpath("Functions/")
addpath("Params/")

num_runs = 10;

MCResults_R = cell(num_runs, 1);
MCResults_P = cell(num_runs, 1);
MCResults_Est = cell(num_runs, 1);

for run=1:num_runs
    simulateEstimationAndControlledCircle_MC;

    MCResults_R{run} = R;
    MCResults_P{run} = P;
    MCResults_Est{run} = Est;

end

%% Plot
plotQuad(MCResults_R, MCResults_P, MCResults_Est)