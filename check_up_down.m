% --- Upright vs Downward check (end-effector Z-axis) ---
% 1) Load FK definitions (creates symbolic r33, q1..q6, etc.)
kinematic_analysis_j2n6s300

% 2) Set joint angles in radians: [q1 q2 q3 q4 q5 q6]
%    NOTE: r33 is already expressed in terms of q1..q6 with built-in offsets,
%    so you should substitute raw joint angles here (no extra offsets).
q_vals = [0 0 0 0 0 0];

% 3) Evaluate r33 = T(3,3) for the given joint configuration
r33_val = double(subs(r33, [q1 q2 q3 q4 q5 q6], q_vals));

% 4) Decide upright vs downward
if r33_val > 0
    disp('Upright (end-effector Z-axis points upward)');
elseif r33_val < 0
    disp('Downward (end-effector Z-axis points downward)');
else
    disp('Neutral (end-effector Z-axis is horizontal)');
end

fprintf('r33 = %.4f\n', r33_val);
