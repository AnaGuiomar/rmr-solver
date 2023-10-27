% Script to run the Rapid Muscle Redundancy (RMR) solver on user-selected experiments.
% The user is prompted with the selection of the tasks to analyze.
% Within the script, it is possible to adjust the downsampling to be
% applied, and whether the analysis should include the glenohumeral
% constraint or not.
%
% Author: Italo Belli (i.belli@tudelft.nl) 2023

close all; clear; clc; beep off;

% Import the OpenSim libraries.
import org.opensim.modeling.*;

% set the path current folder to be the one where this script is contained
mfile_name          = mfilename('fullpath');
[pathstr,~,~]  = fileparts(mfile_name);
cd(pathstr);

% getting path to other folders in this repo
addpath(pathstr)
cd ../../../
path_to_repo = pwd;
addpath(path_to_repo)
addpath(fullfile(path_to_repo, 'Code/Data Processing/'))

%path to OpenSim folder
path_to_opensim = '/Users/guiomarsantoscarvalho/OpenSim/Thesis/UEFS25/';

% where you have the experimental files (.trc)
trc_path = fullfile(path_to_opensim, 'TRC');

% where to save the results
saving_path = fullfile(path_to_opensim, 'RMR');


% Select model
modelFile = append(path_to_opensim, 'TSM_UEFS25_scaled.osim');
model = Model(modelFile);


% Select the experimental data to be considered
dataset_considered = 'Trial2_45W';

[files,path] = uigetfile('*.trc', 'Select the .trc files to analyse', trc_path, 'MultiSelect','on');

if iscell(files)
    num_files = size(files, 2);
else
    num_files = 1;
end

% Set the weight for the various scapula coordinates in IK
% This is to achieve a good agreement between scapula upward rotation and
% shoulder elevation (as reported in the paper)
weight_abd = 0.0001;
weight_elev = 0.0001;
weight_up_rot = 0.0002;
weigth_wing = 0.0001;
weight_coord = [weight_abd, weight_elev, weight_up_rot, weigth_wing];

% Downsampling
time_interval = 10;

% Flags (Select whether to enforce constraints)
dynamic_bounds = true;              % enforcing continuity of the activations from one timestep to the next, to respect first-order dynamics
enforce_GH_constraint = true;       % enforcing directional constraint on the glenohumeral joint force

apply_external_force = 1; % 1 = apply

%Check if 3 extra markers were added to the model (G_center,HH_center,G_edge)
numMarkers = model.getNumMarkers();
assert(numMarkers == 16, 'Add 3 markers:G_center,HH_center,G_edge');

%% Generate the external force and add it to the model
force_params =[];
force_params.apply_external_force =apply_external_force; 
if apply_external_force
    %external_force_point_of_application = 'hand';  % also "thorax" is supported. Here you should input an identifier
                                                    % for the point on which you want to apply the force on. Then, it
                                                    % will be decoded inside the "generate_external_force_3D.m" function
                                                    % so you will have to modify it too

    external_force_filename = append(path_to_opensim, '/ExternalLoads/Treadmill_ExternalForces_45W.xml');             % name of the filename in which the force is going to be stored
    external_force_storagefile = append(path_to_opensim, '/ExternalLoads/UEFS25_ExpTrial_2_4kmh_45W_ExternalForce.sto');
    % 3D components of the force (fictitious for now, you can subsitute
    % this with experimental data)
    % Values are in Newton 
%     external_force_value.x = [0, -5, -10, -20, -40];
%     external_force_value.y = [0];
%     external_force_value.z = [0];
    
    % You can also built a force profile as continuous force curve
    % (here this is exemplified along the X direction of the force
%     way_points = [0, 50, 100, 167, 124, 120, 75, 20, 22, 12];
%     x = linspace(0, (n_steps-1)*0.01, length(way_points));
%     cs = spline(x,way_points);
%     xx = 0:0.01:(n_steps-1)*0.01;
% 
%     external_force.x = ppval(cs, xx);
%     external_force.y = [0];
%     external_force.z = [0];
%     plot(xx, external_force.x);
    
    % Save external force parameters in structure
    %force_params.EF_point_of_application = external_force_point_of_application;
    force_params.EF_filename = external_force_filename;
    force_params.EF_storage_file = external_force_storagefile;
    %force_params.EF = external_force_value;
end

%% Run Rapid Muscle Redundancy (RMR) solver
% preallocating arrays to hold information about the solutions
optimizationStatus = [];
unfeasibility_flag = [];
tOptim = zeros(num_files,1);
result_file_RMR = {};

for trc_file_index=1:num_files
    fprintf('Running RMR on experiment %i \n', trc_file_index)
    if num_files>1
        experiment = append(path, files(trc_file_index));
        experiment = experiment{1};
        has_2kg_weight = str2num(experiment(end-5));      % based on file name
    else
        experiment = append(path,files);
        has_2kg_weight = str2num(experiment(end-5));      % based on file name
    end
    
[aux_optimization_status, aux_unfeasibility_flags, tOptim(trc_file_index), aux_result_file] = RMR_analysis(dataset_considered, model, experiment, 0, weight_coord, time_interval, dynamic_bounds, enforce_GH_constraint, force_params, saving_path);

    optimizationStatus(trc_file_index).experiment = aux_optimization_status;
    result_file_RMR{trc_file_index} = aux_result_file;
    unfeasibility_flag(trc_file_index).experiment = aux_unfeasibility_flags;
    fprintf('\n Solved with %i unfeasible solutions \n \n \n', sum(aux_unfeasibility_flags));
end
