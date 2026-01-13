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
q_range  = [-pi, pi];   % customize as needed
dq_range = [-pi, pi];

q0_list  = (q_range(2)-q_range(1))*rand(num_samples,n) + q_range(1);
dq0_list = (dq_range(2)-dq_range(1))*rand(num_samples,n) + dq_range(1);

%% ----------------------------
% 3. Controller
% -----------------------------
dt = 0.01;          % integrate at 100 Hz
T  = 0:dt:20;

% 你现在 Ts=dt，等价于每一步都更新控制（不需要 hold）
Ts = 0.01;
m = max(1, round(Ts/dt));

kp = 160;
kd = 40;

U_max = 8;          % 若你还想用饱和就保留；不想用可注释

% desired q position
qd_value = 0;
qd_const = [qd_value; qd_value; qd_value; qd_value; qd_value; qd_value];

% -------- NEW: controller-off condition --------
q_limit = 2;      % 角度阈值（rad）。超过就永久关控制器
% ----------------------------------------------

%% ----------------------------
% 4. Simulation Loop
% -----------------------------
U_traj = [];
fail_flag = false(num_samples,1);   % NEW: 记录是否触发关断（失败）

for i = 1:num_samples

    % initialize state
    q = q0_list(i, :)';
    q_dot = dq0_list(i, :)';

    % NEW: latch for controller off
    controller_off = false;

    % storage
    X = [];

    % if you still want hold, initialize it
    U_hold = zeros(n,1);

    for k = 1:length(T)
        % desired trajectory (zero tracking)
        qd = qd_const;
        qd_dot = zeros(n,1);
        qd_ddot = zeros(n,1);

        % store states
        X = [X, [q; qd; q_dot]];

        % ---------- NEW: latch controller off forever ----------
        if ~controller_off && any(abs(q) > q_limit)
            controller_off = true;     % 永久关断
            fail_flag(i) = true;       % 记录失败
            U_hold = zeros(n,1);       % 关断后确保力矩为0
        end
        % ------------------------------------------------------

        % compute control (or set to zero if controller is off)
        if controller_off
            U = zeros(n,1);            % 控制器关掉，但系统继续运行
        else
            if mod(k-1, m) == 0
                V = qd_ddot - kp*(q - qd) - kd*(q_dot - qd_dot);
                D = inertiaMatrix(q);
                g = gravityVector(q);
                U_hold = D*V + g;

                % torque saturation (optional)
                %U_hold = min(max(U_hold, -U_max), U_max);
            end
            U = U_hold;
        end

        % log
        U_traj = [U_traj, U];

        % dynamics update (still running even if U=0)
        q_ddot = inertiaMatrix(q)\(U - gravityVector(q));
        q_dot = q_dot + q_ddot*dt;
        q = q + q_dot*dt;
    end

    % ----------------------------
    % 5. Save results
    % ----------------------------
    q_traj  = X(1:6, :)';
    qd_traj = X(7:12, :)';
    dq_traj = X(13:18, :)';
    time = T';

    save_name = fullfile(save_folder, sprintf('sim_%04d.mat', i));
    save(save_name, 'time', 'q_traj', 'qd_traj', 'dq_traj', 'q0_list', 'dq0_list', ...
        'fail_flag', 'q_limit');

    fprintf('Simulation %d / %d done. Control= %d. \n', i, num_samples, fail_flag(i));
end

disp('All simulations completed and saved.');
