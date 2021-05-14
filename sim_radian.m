clc; clear;

load ER20_1700_kine_fcn.mat

X = deg2rad([10, 3, 10, 10, 5, 4]); % initial state of robo. At this state the robot end is at P1

% These are ideal length value for the robot.
L1 = 504;
L2 = 780;
L3 = 140;
L4 = 760;
L5 = 125;
L8 = 170;

L_ideal = [L1, L2, L3, L4, L5, L8];

L_error = 1;
L1 = L1 + (rand - 0.5) * L_error
L2 = L2 + (rand - 0.5) * L_error
L3 = L3 + (rand - 0.5) * L_error
L4 = L4 + (rand - 0.5) * L_error
L5 = L5 + (rand - 0.5) * L_error
L8 = L8 + (rand - 0.5) * L_error

T6_t_real = DH_T_2(deg2rad(30), 90, 40, 0); % REAL TARGET transformation matrix from robot's end to the STS tool frame.

T_ref_0_real_R = XYZ_Euler(deg2rad(0), deg2rad(0), deg2rad(38.7));
T_ref_0_real_translation = [500; 600; 10.2];

T_ref_0_real = [T_ref_0_real_R, T_ref_0_real_translation; 0, 0, 0, 1];

r_X_t_test = zeros(4, 4, 1 + 3 * 2); %$ (ref: Radian base frame)STS tool frame at P1 , rotate about nominal TCP x_axis,
% rotate about nominal TCP y_axis and rotate about nominal TCP y_axis respectively

r_X_t_test(:, :, 1) = T_ref_0_real * f_T0_6(L1, L2, L3, L4, L5, L8, X(1), X(2), X(3), X(4), X(5), X(6)) * T6_t_real;
disp("r_X_t_test(:, :, 1)=")
disp(r_X_t_test(:, :, 1));

for i = 2:7
    %rotate about the nominal TCP's X (Y or Z) axis; 2 samples for each case.
    X_temp = f_T0_6(L_ideal(1), L_ideal(2), L_ideal(3), L_ideal(4), L_ideal(5), L_ideal(6), X(1), X(2), X(3), X(4), X(5), X(6));

    if (i <= 3)
        angle_to_rot = [deg2rad(45) * (i - 1), 0, 0];
    elseif (i <= 5)
        angle_to_rot = [0, deg2rad(45) * (i - 3), 0];
    elseif (i <= 7)
        angle_to_rot = [0, 0, deg2rad(45) * (i - 5)];
    end

    X_rot_temp = X_temp(1:3, 1:3) * XYZ_Euler(angle_to_rot(1), angle_to_rot(2), angle_to_rot(3));
    X_temp(1:3, 1:3) = X_rot_temp;
    X_plan = efort_inv2(X_temp, L_ideal(1), L_ideal(2), L_ideal(3), L_ideal(4), L_ideal(5), L_ideal(6), f_T0_3, f_T0_4); % get inv sol with ideal DH param
    [~, idx] = min(sum(abs(X_plan(:, :, 1) - X), 2)); % sum(X,2), sum along each row's elements
    X_choose = X_plan(idx, :)
    X6_actual = f_T0_6(L1, L2, L3, L4, L5, L8, X_choose(1), X_choose(2), X_choose(3), X_choose(4), X_choose(5), X_choose(6));

    % efort_inv2(X_temp, L1, L2, L3, L4, L5, L8, f_T0_3, f_T0_4); % disp if there is a valid inverse sol.
    r_X_t_test(:, :, i) = T_ref_0_real * X6_actual * T6_t_real;
    disp("r_X_t_test(:, :, i) = ");
    disp(r_X_t_test(:, :, i));
end

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

r_X_6_real = T_ref_0_real * f_T0_6(L1, L2, L3, L4, L5, L8, X(1), X(2), X(3), X(4), X(5), X(6))

r_R_6 = [r_x6_test, r_y6_test, r_z6_test]

% p6_t = r_X_t_test(1:3, 4, 2) - r_X_t_test(1:3, 4, 2)

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
r_x6_p = circlefit3d(r_X_t_test(1:3, 4, 1)', r_X_t_test(1:3, 4, 2)', r_X_t_test(1:3, 4, 3)');
r_x6_p = r_x6_p';
r_y6_p = circlefit3d(r_X_t_test(1:3, 4, 1)', r_X_t_test(1:3, 4, 4)', r_X_t_test(1:3, 4, 5)');
r_y6_p = r_y6_p';

[dis, r_o6_assume1, ~] = find_dist_between_3d_lines(r_z6_p, r_z6_assume, r_x6_p, r_x6_assume)
[dis, r_o6_assume2, ~] = find_dist_between_3d_lines(r_z6_p, r_z6_assume, r_y6_p, r_y6_assume)

disp(r_o6_assume1 - r_o6_assume2) % show the error

p6_t_assume = transpose(r_R_6_assume) * (r_X_t_test(1:3, 4, 1) - r_o6_assume1)

T6_t_assume = [R6_t_assume, p6_t_assume; 0, 0, 0, 1]

% T6_t_assume

%% Find r_T_0. rotate joint 1 only. get 4 samples. (90 degree each time)
T06_ideal = zeros(4, 4, 4); % get through param before calibration
Tr_t_measured = zeros(4, 4, 4); % measured through Radian
Tr_0_assume = zeros(4, 4, 4);

X = deg2rad([0, 3, 10, 10, 5, 4]);

for i = 1:4

    T06_ideal(:, :, i) = f_T0_6(L_ideal(1), L_ideal(2), L_ideal(3), L_ideal(4), L_ideal(5), L_ideal(6), X(1), X(2), X(3), X(4), X(5), X(6));
    Tr_t_measured(:, :, i) = T_ref_0_real * f_T0_6(L1, L2, L3, L4, L5, L8, X(1), X(2), X(3), X(4), X(5), X(6)) * T6_t_real;

    Tr_6_assume = Tr_t_measured(:, :, i) * inv(T6_t_assume);

    Tr_0_assume(:, :, i) =  Tr_6_assume * inv(T06_ideal(:, :, i));

    if i <= 3
        X(1) = X(1) + deg2rad(90);
    end
end
