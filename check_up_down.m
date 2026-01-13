% --- Upright vs Downward check (no file edits needed) ---
% 1) Load FK definitions (creates r33, etc.)
kinematic_analysis_j2n6s300

% 2) Use your angle a (example: a = pi)
a = 0;

% 3) Evaluate r33 = T(3,3) for q = [a a a a a a]
r33_val = double(subs(r33, [q1 q2 q3 q4 q5 q6], [a a a a a a]));

% 4) Decide upright vs downward
if r33_val > 0
    disp('Upright (end-effector Z-axis points upward)');
elseif r33_val < 0
    disp('Downward (end-effector Z-axis points downward)');
else
    disp('Neutral (end-effector Z-axis is horizontal)');
end

fprintf('r33 = %.4f\n', r33_val);
