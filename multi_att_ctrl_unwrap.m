clc; clear; close all;

%% ----------------------------
% 1. Configuration
% -----------------------------
num_samples = 50;
save_folder = fullfile(pwd, 'sim_database_100Hz');
if ~exist(save_folder, 'dir'); mkdir(save_folder); end

%% ----------------------------
% 2. Random initial conditions
% -----------------------------
n = 6; % 6 DOF
q_range  = [0, 6.00];      % rad
dq_range = [-5.0, 5.0];    % rad/s

q0_list  = (q_range(2)-q_range(1))*rand(num_samples,n) + q_range(1);
dq0_list = (dq_range(2)-dq_range(1))*rand(num_samples,n) + dq_range(1);

%% ----------------------------
% 3. Controller / Simulation setup
% -----------------------------
dt = 0.01;                 % integration at 100 Hz
T  = 0:dt:20;
N  = numel(T);

Ts = 0.02;                 % control update at 25 Hz
m  = max(1, round(Ts/dt)); % integration steps per control update

kp = 10;
kd = 4;

U_max = 8;                 % saturation limit (per joint)

qd_const = [pi; pi; pi; pi; pi; pi];  % desired joint angles
angle_wrap = true;         % true: use shortest angle error (for revolute joints)

%% ----------------------------
% 4. Simulation Loop
% -----------------------------
for i = 1:num_samples
    fprintf('Running simulation %d / %d...\n', i, num_samples);

    % initial state
    q     = q0_list(i, :)';
    q_dot = dq0_list(i, :)';

    % preallocate storage
    q_traj    = zeros(N, n);
    dq_traj   = zeros(N, n);
    qd_traj   = repmat(qd_const(:)', N, 1);

    u_raw_traj = zeros(N, n);
    u_traj     = zeros(N, n);

    U_hold = zeros(n,1);

    for k = 1:N
        % store
        q_traj(k,:)  = q';
        dq_traj(k,:) = q_dot';

        % desired
        qd      = qd_const;
        qd_dot  = zeros(n,1);
        qd_ddot = zeros(n,1);

        % update control only every Ts
        if mod(k-1, m) == 0
            % ----- PD error -----
            if angle_wrap
                % shortest angle error: e = wrapToPi(q - qd) without relying on toolbox
                e = atan2(sin(q - qd), cos(q - qd));
            else
                e = (q - qd);
            end

            edot = (q_dot - qd_dot);

            % computed-torque style PD (no C term in your original code)
            V = qd_ddot - kp * e - kd * edot;

            D = inertiaMatrix(q);
            g = gravityVector(q);

            U_raw = D*V + g;

            % saturation
            U_hold = max(min(U_raw, U_max), -U_max);
        end

        % hold control between updates
        U = U_hold;

        % log u
        u_traj(k,:) = U';
        % 如果你想同时记录每次更新的 raw_u（没有更新时用上一次值）
        % 这里简单做成每步都存“当前 hold 的值”，更利于画图
        u_raw_traj(k,:) = U';  % 若想存真正U_raw可在if里另存一个U_raw_hold

        % ----- dynamics update -----
        % 建议：至少把重力项和惯性项保持一致
        q_ddot = inertiaMatrix(q) \ (U - gravityVector(q));
        % 如果你有 cor_centriTerms(q,q_dot)，强烈建议加回去：
        % q_ddot = inertiaMatrix(q) \ (U - gravityVector(q) - cor_centriTerms(q, q_dot));

        q_dot = q_dot + q_ddot * dt;
        q     = q     + q_dot  * dt;
    end

    % save per-simulation results
    time = T';
    save_name = fullfile(save_folder, sprintf('sim_%04d.mat', i));
    save(save_name, 'time', 'q_traj', 'qd_traj', 'dq_traj', ...
                    'u_traj', 'u_raw_traj', 'U_max', 'kp', 'kd', ...
                    'q0_list', 'dq0_list', 'Ts', 'dt', 'angle_wrap');
end

disp('All simulations completed and saved.');
