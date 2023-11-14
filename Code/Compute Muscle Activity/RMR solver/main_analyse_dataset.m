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

%choose participant & Trial
participant = 'UEFS08';
dataset_considered = 'Trial2_45W';

%path to OpenSim folder
path_to_opensim = ['/Users/guiomarsantoscarvalho/OpenSim/Thesis/', participant,'/'];

% Select model
modelFile = append(path_to_opensim, 'TSM_', participant,'_scaled.osim');
model = Model(modelFile);

% where you have the experimental files (.trc)
% trc_path = fullfile(path_to_opensim, 'TRC');
% [files,path] = uigetfile('*.trc', 'Select the .trc files to analyse', trc_path, 'MultiSelect','on');
% experiment = append(path,files);
experiment = 0;         %No TRC file

% where to save the results
saving_path = fullfile(path_to_opensim, 'RMR');

% get the motion file from Scaling 
motion_file = fullfile([path_to_opensim, '/ScalingResults/Corrected Markers/'] , [participant, '_Trial2_45W_IK_fitted_corrected.mot']);
% motion_file = fullfile(saving_path , 'IK_UEFS08_ExpTrial_2_4kmh_45W.mot');
% motion_file = 0; % No motionfile

% Downsampling
time_interval = 30;

% Flags (Select whether to enforce constraints)
dynamic_bounds = true;               % enforcing continuity of the activations from one timestep to the next, to respect first-order dynamics
enforce_GH_constraint = true;       % enforcing directional constraint on the glenohumeral joint force
apply_external_force = 1;

%Check if 3 extra markers were added to the model (G_center,HH_center,G_edge)
numMarkers = model.getNumMarkers();
assert(numMarkers == 16, 'Add 3 markers:G_center,HH_center,G_edge');

%% Generate the external force and add it to the model
force_params =[];
force_params.apply_external_force = apply_external_force;
force_1 = [];
% Create an empty torque identifier
torque_identifier = '';

if apply_external_force

    external_force_filename = 'ExternalForces_Trial2_45W.xml';             % name of the filename in which the force is going to be stored
    
    % Create Storage object
    output_file_path = fullfile(path_to_opensim, 'ExternalLoads', [participant '_ExpTrial_2_4kmh_45W_ExternalForce.sto']);
    data_storage = Storage(output_file_path);
    data_storage.setName([participant '_ExpTrial_2_4kmh_45W_ExternalForce.sto']);
    
    % Create ExternalForce object
    external_force = ExternalForce(data_storage, "ground_force_v", "ground_force_p", torque_identifier, "hand", "ground", "ground");
    external_force.print(fullfile(path_to_opensim, 'ExternalLoads', external_force_filename));

    % Save external force parameters in structure
    force_1.ef_filename = external_force_filename;
    force_1.ef_storage = data_storage;
    force_1.ef = external_force;

    % Update force_params structure
    force_params.num_forces = 1;
    force_params.forces{1} = force_1;
end

%% Run Rapid Muscle Redundancy (RMR) solver
disp('Running RMR')

[optimization_status, unfeasibility_flags, tOptim, result_file] = RMR_analysis(participant, model, experiment, motion_file, [], time_interval, dynamic_bounds, enforce_GH_constraint, force_params, saving_path);

fprintf('\n Solved with %i unfeasible solutions \n \n \n', sum(unfeasibility_flags));
