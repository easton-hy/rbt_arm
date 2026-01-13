clc; clear; close all;

%% ----------------------------
% 1. Configuration
% -----------------------------
num_samples = 50;                    % number of simulations
save_folder = fullfile(pwd, 'sim_database_100Hz');
if ~exist(save_folder, 'dir')
    mkdir(save_folder);
end

%% ----------------------------
% 2. Random initial conditions
% -----------------------------

n = 6; % 6 DOF system
q_range  = [-3, 3];   % customize as needed
dq_range = [-20, 20];

q0_list  = (q_range(2)-q_range(1))*rand(num_samples,n) + q_range(1);
dq0_list = (dq_range(2)-dq_range(1))*rand(num_samples,n) + dq_range(1);
% 
% % load the initial state
% q0_list = load('q0_all.mat').q0_all;
% dq0_list = load('dq0_all.mat').dq0_all;

%% ----------------------------
% 3. Controller
% -----------------------------

dt = 0.01;          % integrate at 100 Hz (or faster)
T  = 0:dt:30;

Ts = 0.01;          % control at 20 Hz
m = round(Ts/dt);   % how many integration steps per control update
U_hold = zeros(n,1);

kp = 10;  %10   6
kd = 6; %4    2

U_max = 8;

% desired q position
qd_const = [0; 0; 0; 0; 0; 0];

%% ----------------------------
% 3. Simulation Loop
% -----------------------------
U_traj = []; 
for i = 1:num_samples
    fprintf('Running simulation %d / %d...\n', i, num_samples);

    % initialize state
    q = q0_list(i, :)';
    q_dot = dq0_list(i, :)';

    % storage
    X = [];

    for k = 1:length(T)
        t = T(k);

        % desired trajectory (zero tracking)
        qd = qd_const;
        qd_dot = zeros(n,1);
        qd_ddot = zeros(n,1);

        % store states
        X = [X, [q; qd; q_dot]];

%         % PD control
%         V = qd_ddot - kp*(q - qd) - kd*(q_dot - qd_dot);
%         D = 1.0 * inertiaMatrix(q);
%         g = 1.0 * gravityVector(q);
%         U = D*V + g;

        if mod(k-1, m) == 0
            % compute control only every Ts
            V = qd_ddot - kp*(q - qd) - kd*(q_dot - qd_dot);
            D = inertiaMatrix(q);
            g = gravityVector(q);
            U_hold = D*V + g;
            U_hold = min(max(U_hold, -U_max), U_max);
        end
    
        U = U_hold;  % hold control between updates
        U_traj = [U_traj, U];

        % dynamics update
        %q_ddot = inertiaMatrix(q) \ (U - gravityVector(q) - cor_centriTerms(q, q_dot));
        q_ddot = inertiaMatrix(q)\(U -gravityVector(q));
        q_dot = q_dot + q_ddot*dt;
        q = q + q_dot*dt;
    end

    % ----------------------------
    % 4. Save results
    % ----------------------------
    q_traj  = X(1:6, :)';
    qd_traj = X(7:12, :)';
    dq_traj = X(13:18, :)';
    time = T';

    save_name = fullfile(save_folder, sprintf('sim_%04d.mat', i));
    save(save_name, 'time', 'q_traj', 'qd_traj', 'dq_traj', 'q0_list', 'dq0_list');
end

disp('All simulations completed and saved.');
