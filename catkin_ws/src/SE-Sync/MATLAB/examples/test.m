% This file contains a minimum working example demonstrating the use of the
% MATLAB distribution of SE-Sync, a certifiably correct algorithm for
% synchronization over the special Euclidean group
%
% Copyright (C) 2016 by David M. Rosen

%% Reset environment
clear all;
close all;
clc;

%% Import SE-Sync
run('/home/fregu856/AA273/AA273_project/catkin_ws/src/SE-Sync/MATLAB/import_SE_Sync.m');  % It's that easy :-)!


%% Select dataset to run
data_dir = '/home/fregu856/AA273/AA273_project/catkin_ws/src/SE-Sync/data/'  % Relative path to directory containing example datasets

% 3D datasets
sphere2500 = 'sphere2500';
torus = 'torus3D';
grid = 'grid3D';
garage = 'parking-garage';
cubicle = 'cubicle';
rim = 'rim';

% 2D datasets
CSAIL = 'CSAIL';
manhattan = 'manhattan';
city10000 = 'city10000';
intel = 'intel';
ais = 'ais2klinik';


% Pick the dataset to run here
file = 'graph2';

g2o_file = strcat(data_dir, file, '.g2o');

%% Read in .g2o file
tic();
fprintf('Loading file: %s ...\n', g2o_file);
measurements = load_g2o_data(g2o_file);
t = toc();
num_poses = max(max(measurements.edges));
num_measurements = length(measurements.kappa);
d = length(measurements.t{1});
fprintf('Processed input file %s in %g seconds\n', g2o_file, t);
fprintf('Number of poses: %d\n', num_poses);
fprintf('Number of measurements: %d\n', num_measurements);

%% Set Manopt options (if desired)
Manopt_opts.tolgradnorm = 1e-2;  % Stopping tolerance for norm of Riemannian gradient
Manopt_opts.rel_func_tol = 1e-5;  % Additional stopping criterion for Manopt: stop if the relative function decrease between two successive accepted iterates is less than this value
Manopt_opts.miniter = 1;  % Minimum number of outer iterations (i.e. accepted update steps) to perform
Manopt_opts.maxiter = 300;  % Maximum number of outer iterations (i.e. accepted update steps) to perform
Manopt_opts.maxinner = 500;  % Maximum number of iterations for the conjugate-gradient method used to compute approximate Newton steps
%manopt_options.maxtime = 60*60;  % Maximum computation time to allow, in seconds
%manopt_options.solver = @steepestdescent;  % Select Manopt solver to use: {trustregions (default), conjugategradient, steepestdescent}


%% Set SE-Sync options (if desired)
SE_Sync_opts.r0 = 5;  % Initial maximum-rank parameter at which to start the Riemannian Staircase
SE_Sync_opts.rmax = 10;  % Maximum maximum-rank parameter at which to terminate the Riemannian Staircase
SE_Sync_opts.eig_comp_rel_tol = 1e-4;  % Relative tolerance for the minimum-eigenvalue computation used to test for second-order optimality with MATLAB's eigs() function
SE_Sync_opts.min_eig_lower_bound = -1e-3;  % Minimum eigenvalue threshold for accepting a maxtrix as numerically positive-semidefinite
SE_Sync_opts.Cholesky = false;  % Select whether to use Cholesky or QR decomposition to compute orthogonal projections

use_chordal_initialization = true;  % Select whether to use the chordal initialization, or a random starting point

%% Run SE-Sync

% Pass explict settings for SE-Sync and/or Manopt, and use chordal
% initialization
fprintf('Computing chordal initialization...\n');
R = chordal_initialization(measurements);
Y0 = vertcat(R, zeros(SE_Sync_opts.r0 - d, num_poses*d));
[SDPval, Yopt, xhat, Fxhat, SE_Sync_info, problem_data] = SE_Sync(measurements, Manopt_opts, SE_Sync_opts, Y0);

% ... or ...

% Use default settings for everything
%[SDPval, Yopt, xhat, Fxhat, se_sync_info, problem_data] = SE_Sync(measurements);

D = 2;

t_hat = xhat.t;
Rhat = xhat.R;

% 'Unrotate' these vectors by premultiplying by the inverse of the first
% orientation estimate
t_hat_rotated = Rhat(1:D, 1:D)' * t_hat;
% Translate the resulting vectors to the origin
t_hat_anchored = t_hat_rotated - repmat(t_hat_rotated(:, 1), 1, size(t_hat_rotated, 2));

thetas = zeros(length(xhat.t), 1);
for i = 1:length(xhat.t)
    R = xhat.R(:, 2*(i-1)+1:2*(i-1)+2);
    cos_th = R(1,1);
    sin_th = R(2,1);
    theta = atan2(sin_th, cos_th);
    thetas(i) = theta;
end

thetas_anchored = thetas - thetas(1);

poses = zeros(3, length(xhat.t));
poses(1:2, :) = xhat.t;
poses(3,:) = thetas;

fileID = fopen('test.txt','w');
fprintf(fileID,'%12.12f %12.12f %12.12f\n', poses);
fclose(fileID);

% each line in test.txt is x y theta
