% File: run_multiple_initial_states.m
% Purpose: Run Simulink model for multiple initial states
%          and save each simulation's trajectory as one .mat file

clc; clear; close all;

% ----------------------------
% 1. Configuration
% ----------------------------
model_name = 'pd_controller_j2n6s300';   % Your Simulink model name
num_samples = 200;                      % Number of initial states
save_folder = fullfile(pwd, 'sim_database');  % Folder to save results
stop_time = '0.2';                        % Simulation stop time (s)

% Create folder if not exist
if ~exist(save_folder, 'dir')
    mkdir(save_folder);
end

% ----------------------------
% 2. Generate initial conditions
% ----------------------------
% Example: 6-joint system
n = 6;
q0_list  = 2*pi*rand(num_samples, n) - pi;  % Range [-pi, pi]
dq0_list = 20*rand(num_samples, n) - 10;   % Range [-10, 10]

% ----------------------------
% 3. Simulation loop
% ----------------------------
for i = 1:num_samples
    fprintf('Running simulation %d / %d...\n', i, num_samples);

    % Get initial state
    q0  = q0_list(i, :).';
    dq0 = dq0_list(i, :).';

    % Update subsystem parameters
    set_param([model_name, '/dynamic system'], ...
              'phi0', mat2str(q0), ...
              'dphi0', mat2str(dq0));

    % Run simulation (assuming Fast Restart is ON)
    simOut = sim(model_name, ...
        'SimulationMode', 'normal', ...
        'StopTime', stop_time, ...
        'SaveOutput', 'on', ...
        'SaveState', 'off', ...
        'ReturnWorkspaceOutputs', 'on');

    % ----------------------------
    % 4. Extract simulation results
    % ----------------------------
    dq_ts = simOut.get('dq'); 
    q_ts  = simOut.get('q');

    dq = squeeze(dq_ts.Data);      
    q  = squeeze(q_ts.Data);  
    t  = dq_ts.Time;      

    % ----------------------------
    % 5. Save results to .mat file
    % ----------------------------
    save_name = fullfile(save_folder, sprintf('sim_%04d.mat', i));
    save(save_name, 'q', 'dq', 'q0', 'dq0');

end

disp('✅ All simulations completed and saved.');
