clc; clear;

load ER20_1700_kine_fcn.mat

% These are ideal length value for the robot.
L_ideal = [504, 780, 140, 760, 125, 170];

max_delta_theta = deg2rad(1 * ones(1, 6)); %maximum offset: 1 degree.
max_delta_d = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1];
max_delta_a = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1];
max_delta_alpha = deg2rad(0.5 * ones(1, 5)); % maximum : 0.5 degree's deviation

actual_delta_theta = (rand(1, 6) - 0.5) .* max_delta_theta
actual_delta_d = (rand(1, 6) - 0.5) .* max_delta_d
actual_delta_a = (rand(1, 6) - 0.5) .* max_delta_a
actual_delta_alpha = (rand(1, 5) - 0.5) .* max_delta_alpha

actual_T0_1 = eval(simplify(DH_T_2(x1 + actual_delta_theta(1), L1 + actual_delta_d(1), L8 + actual_delta_a(1), pi / 2 + actual_delta_alpha(1))));
actual_T1_2 = eval(simplify(DH_T_2(x2 - pi / 2 + actual_delta_theta(2), 0 + actual_delta_d(2), -L2 + actual_delta_a(2), 0 + actual_delta_alpha(2))));
actual_T2_3 = eval(simplify(DH_T_2(x3 + pi + actual_delta_theta(3), 0 + actual_delta_d(3), L3 + actual_delta_a(3), pi / 2 + actual_delta_alpha(3))));
actual_T3_4 = eval(simplify(DH_T_2(x4 + actual_delta_theta(4), L4 + actual_delta_d(4), 0 + actual_delta_a(4), -pi / 2 + actual_delta_alpha(4))));
actual_T4_5 = eval(simplify(DH_T_2(x5 + actual_delta_theta(5), 0 + actual_delta_d(5), 0 + actual_delta_a(5), pi / 2 + actual_delta_alpha(5))));
actual_T5_6 = eval(simplify(DH_T_2(x6 + actual_delta_theta(6), L5 + actual_delta_d(6), 0 + actual_delta_a(6), 0)));

actual_T0_6 = actual_T0_1 * actual_T1_2 * actual_T2_3 * actual_T3_4 * actual_T4_5 * actual_T5_6;

actual_f_T0_6 = matlabFunction(actual_T0_6);

theta_series_bound = deg2rad([-45, 45; ...
                            - 10, 30; ...
                            -20, 30; ...
                            -60, 60; ...
                            -15, 15; ...
                            -45, 45]);
nn = 48;
joint_target_series = zeros(6, 3 * nn); % 18 sets of test joint pos.
theta123_target = zeros(3, nn);


% method 1, will lead C
% for i = 1:3
%     theta123_target(i, :) = linspace(theta_series_bound(1, 1), theta_series_bound(1, 2), nn);
% end

% for i = 1:3 % set theta1 2 and 3
%     joint_target_series(1, i:3:end) = theta123_target(1, :);
%     joint_target_series(2, i:3:end) = theta123_target(2, :);
%     joint_target_series(3, i:3:end) = theta123_target(3, :);
% end

% for i = 1:3:3 * nn
%     joint_target_series(4, i:i + 2) = linspace(theta_series_bound(4, 1), theta_series_bound(4, 2), 3);
%     joint_target_series(5, i:i + 2) = linspace(theta_series_bound(5, 1), theta_series_bound(5, 2), 3);
%     joint_target_series(6, i:i + 2) = linspace(theta_series_bound(6, 1), theta_series_bound(6, 2), 3);
% end

% METHOD 2, RANDOM JOINT POS
for i = 1:3 * nn
    joint_target_series(1:6,i) = theta_series_bound(1:6,1) + rand(6,1).*(theta_series_bound(1:6,2) - theta_series_bound(1:6,1));
end

C_combined = zeros(6*3*nn,23);
delta_X_combined = zeros(6*3*nn,1);

for i=1:3*nn % test actual and ideal end pose
    X = joint_target_series(:,i);
    actual_T0_6_val = actual_f_T0_6(L_ideal(1),L_ideal(2),L_ideal(3),L_ideal(4),L_ideal(5),L_ideal(6),X(1),X(2),X(3),X(4),X(5),X(6));
    ideal_T0_6_val = f_T0_6(L_ideal(1),L_ideal(2),L_ideal(3),L_ideal(4),L_ideal(5),L_ideal(6),X(1),X(2),X(3),X(4),X(5),X(6));
    % D_T0_6 = actual_T0_6_val * inv(ideal_T0_6_val)
    delta_P = actual_T0_6_val(1:3,4) - ideal_T0_6_val(1:3,4);
    temp_delta_R =  actual_T0_6_val(1:3,1:3) * transpose(ideal_T0_6_val(1:3,1:3)); % 0 frame as the ref frame 
    delta_orientation = [temp_delta_R(3,2); temp_delta_R(1,3); temp_delta_R(2,1)] % see page 72

  
    delta_X_combined(1 + (i-1)*6: 1 + (i-1)*6 + 5, 1) = [delta_P*1e-3; delta_orientation]; % unit : m and rad
    C_combined(1 + (i-1)*6: 1 + (i-1)*6 + 5, :) = C_fcn(L_ideal(2),L_ideal(3),L_ideal(4),L_ideal(5),L_ideal(6),X(1),X(2),X(3),X(4),X(5),X(6));
end

C_combined(:,9) = []; % the J_d's rank is always less than 6 because z1 and z2 axises are paralel. Thus, let d23 = d2 + d3.
d_DH_param = inv(transpose(C_combined)* C_combined)*transpose(C_combined)*delta_X_combined
 
actual_d_DH_param = [actual_delta_theta, actual_delta_d, actual_delta_a, actual_delta_alpha]';
actual_d_DH_param(8) = actual_d_DH_param(8) + actual_d_DH_param(9);
actual_d_DH_param(9) = [];
actual_d_DH_param