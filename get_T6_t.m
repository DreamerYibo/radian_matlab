clc; clear;

load ER20_1700_kine_fcn.mat
load robo1_step1_data.mat

X = deg2rad([10, 3, 10, 10, 5, 4]); % initial state of robo. At this state the robot end is at P1
r_X_t_test = zeros(4, 4, 1 + 3 * 2); %$ (ref: Radian base frame)STS tool frame at P1 , rotate about nominal TCP x_axis,

tempX = zeros(4, 3, 15);
tempX_homo = zeros(4, 4, 15);

for i = 1:15
    tempX(:, :, i) = robo1_get_T6t_data(4 * (i - 1) + 1:4 * (i - 1) + 4, :);
    tempX_homo(1:3, 1:3, i) = transpose(tempX(2:4, :, i));
    tempX_homo(1:3, 4, i) = transpose(tempX(1, :, i));
    tempX_homo(4, :, i) = [0, 0, 0, 1];
end

r_X_t_test(:, :, 1) = tempX_homo(:, :, 1);
r_X_t_test(:, :, 2) = tempX_homo(:, :, 2);
r_X_t_test(:, :, 3) = tempX_homo(:, :, 4);
r_X_t_test(:, :, 4) = tempX_homo(:, :, 6);
r_X_t_test(:, :, 5) = tempX_homo(:, :, 8);
r_X_t_test(:, :, 6) = tempX_homo(:, :, 10);
r_X_t_test(:, :, 7) = tempX_homo(:, :, 11);

disp("r_X_t_test(:, :, 1)=")
disp(r_X_t_test(:, :, 1))

r_x6_test = zeros(3, 1);
r_y6_test = zeros(3, 1);
r_z6_test = zeros(3, 1);

delta_R = transpose(r_X_t_test(1:3, 1:3, 1)) * r_X_t_test(1:3, 1:3, 2);
phi = acos((trace(delta_R) - 1) / 2); %Equivalent rotation angle about axis k from Re_0 to Re_f .
k_axis = (1 / (2 * sin(phi))) * [delta_R(3, 2) - delta_R(2, 3); ...
                                delta_R(1, 3) - delta_R(3, 1); delta_R(2, 1) - delta_R(1, 2)];
r_x6_test = r_X_t_test(1:3, 1:3, 1) * k_axis % remember to multiply k_axis by r_X_t_test(1:3,1:3,1) to change the ref coordinate.

delta_R = transpose(r_X_t_test(1:3, 1:3, 1)) * r_X_t_test(1:3, 1:3, 4);
phi = acos((trace(delta_R) - 1) / 2); %Equivalent rotation angle about axis k from Re_0 to Re_f .
k_axis = (1 / (2 * sin(phi))) * [delta_R(3, 2) - delta_R(2, 3); ...
                                delta_R(1, 3) - delta_R(3, 1); delta_R(2, 1) - delta_R(1, 2)];
r_y6_test = r_X_t_test(1:3, 1:3, 1) * k_axis

delta_R = transpose(r_X_t_test(1:3, 1:3, 1)) * r_X_t_test(1:3, 1:3, 6);
phi = acos((trace(delta_R) - 1) / 2); %Equivalent rotation angle about axis k from Re_0 to Re_f .
k_axis = (1 / (2 * sin(phi))) * [delta_R(3, 2) - delta_R(2, 3); ...
                                delta_R(1, 3) - delta_R(3, 1); delta_R(2, 1) - delta_R(1, 2)];
r_z6_test = r_X_t_test(1:3, 1:3, 1) * k_axis

r_R_6 = [r_x6_test, r_y6_test, r_z6_test];

% NOTE r_z6_test should be the most accurate one. Because rotation about z6 only requires the movement of joint6.
% adjust r_x6_test and r_y6_test based on r_z6_test.

r_z6_assume = r_z6_test;

r_y6_assume = cross(r_z6_assume, r_x6_test);
r_y6_assume = r_y6_assume ./ norm(r_y6_assume);

r_x6_assume = cross(r_y6_assume, r_z6_assume);

error_r_x6 = cross(r_x6_assume, r_x6_test)
error_r_y6 = cross(r_y6_assume, r_y6_test)

r_R_6_assume = [r_x6_assume, r_y6_assume, r_z6_assume];
R6_t_assume = transpose(r_R_6_assume) * r_X_t_test(1:3, 1:3, 1) % assume R6_t

% find the assumed original point of o6.

r_z6_p = circlefit3d(transpose(r_X_t_test(1:3, 4, 1)), transpose(r_X_t_test(1:3, 4, 6)), transpose(r_X_t_test(1:3, 4, 7)));
r_z6_p = r_z6_p';
r_x6_p = circlefit3d(r_X_t_test(1:3, 4, 2)', r_X_t_test(1:3, 4, 1)', r_X_t_test(1:3, 4, 3)');
r_x6_p = r_x6_p';
r_y6_p = circlefit3d(r_X_t_test(1:3, 4, 4)', r_X_t_test(1:3, 4, 1)', r_X_t_test(1:3, 4, 5)');
r_y6_p = r_y6_p';

[dis, r_o6_assume1, ~] = find_dist_between_3d_lines(r_z6_p, r_z6_assume, r_x6_p, r_x6_assume)
[dis, r_o6_assume2, ~] = find_dist_between_3d_lines(r_z6_p, r_z6_assume, r_y6_p, r_y6_assume)

disp(r_o6_assume1 - r_o6_assume2) % show the error

p6_t_assume = transpose(r_R_6_assume) * (r_X_t_test(1:3, 4, 1) - r_o6_assume1)

T6_t_assume = [R6_t_assume, p6_t_assume; 0, 0, 0, 1]

% T6_t_assume

%% Find r_T_0. rotate joint 1 only. get 4 samples. (90 degree each time)
% T06_ideal = zeros(4, 4, 4); % get through param before calibration
% Tr_t_measured = zeros(4, 4, 4); % measured through Radian
% Tr_0_assume = zeros(4, 4, 4);

% X = deg2rad([0, 3, 10, 10, 5, 4]);

% for i = 1:4

%     T06_ideal(:, :, i) = f_T0_6(L_ideal(1), L_ideal(2), L_ideal(3), L_ideal(4), L_ideal(5), L_ideal(6), X(1), X(2), X(3), X(4), X(5), X(6));
%     Tr_t_measured(:, :, i) = T_ref_0_real * f_T0_6(L1, L2, L3, L4, L5, L8, X(1), X(2), X(3), X(4), X(5), X(6)) * T6_t_real;

%     Tr_6_assume = Tr_t_measured(:, :, i) * inv(T6_t_assume);

%     Tr_0_assume(:, :, i) = Tr_6_assume * inv(T06_ideal(:, :, i));

%     if i <= 3
%         X(1) = X(1) + deg2rad(90);
%     end

% end
