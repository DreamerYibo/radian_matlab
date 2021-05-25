function [C_fcn_handle, mod_T06_fcn_handle] = get_C_fcn_and_T06(d_DH_param)
    % d_DH_param is d_theta1 to d_theta6; d_d1, d_d2_3, d_d4 to d_d46;d_a1 to d_a6; d_alpha1 to d_alpha2

    if (length(d_DH_param) ~= 22)
        error("Input d_DH_param is d_theta1 to d_theta6; d_d1, d_d2_3, d_d4 to d_d46;d_a1 to d_a6; d_alpha1 to d_alpha2")
    end

    delta_theta = d_DH_param(1:6);
    delta_d = d_DH_param(7:11)*1e3; %%IMPORTANT!!! Remember to convert this unit from m to mm!!!!!!
    delta_a = d_DH_param(12:17)*1e3;
    delta_alpha = d_DH_param(18:22);

    syms x1 x2 x3 x4 x5 x6; % x stands for theta.
    syms L1 L2 L3 L4 L5 L8

    T0_1 = DH_T_2(x1 + delta_theta(1), L1 + delta_d(1), L8 + delta_a(1), pi / 2 + delta_alpha(1));
    T1_2 = DH_T_2(x2 - pi / 2 + delta_theta(2), 0 + delta_d(2), -L2 + delta_a(2), 0 + delta_alpha(2));
    T2_3 = DH_T_2(x3 + pi + delta_theta(3), 0, L3 + delta_a(3), pi / 2 + delta_alpha(3));
    T3_4 = DH_T_2(x4 + delta_theta(4), L4 + delta_d(3), 0 + delta_a(4), -pi / 2 + delta_alpha(4));
    T4_5 = DH_T_2(x5 + delta_theta(5), 0 + delta_d(4), 0 + delta_a(5), pi / 2 + delta_alpha(5));
    T5_6 = DH_T_2(x6 + delta_theta(6), L5 + delta_d(5), 0 + delta_a(6), 0);

    T0_6 = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6;

 
    T0_2 = T0_1 * T1_2;

    T0_3 = T0_2 * T2_3;

    T0_4 = T0_3 * T3_4;

    T0_5 = T0_4 * T4_5;

    mod_T06_fcn_handle = matlabFunction(T0_6); % get mod_T06_fcn_handle

    syms J Jv J_omega ksi1 ksi2 ksi3 ksi4 ksi5 ksi6 rho1 rho2 rho3 rho4 rho5 rho6 % Get the Jaccobian mat of the robot. Choose the base frame 0 as the reference
    syms J_d J_a J_alpha rho_temp ksi_temp % similar definition as the jacobian for joint position

    ksi1 = [0; 0; 1];
    ksi2 = T0_1(1:3, 3);
    ksi3 = T0_2(1:3, 3);
    ksi4 = T0_3(1:3, 3);
    ksi5 = T0_4(1:3, 3);
    ksi6 = T0_5(1:3, 3);

    rho1 = T0_6(1:3, 4);
    rho2 = T0_6(1:3, 4) - T0_1(1:3, 4);
    rho3 = T0_6(1:3, 4) - T0_2(1:3, 4);
    rho4 = T0_6(1:3, 4) - T0_3(1:3, 4);
    rho5 = T0_6(1:3, 4) - T0_4(1:3, 4);
    rho6 = [0; 0; 0]; % NOTE this!

    Jv = [cross(ksi1, rho1), cross(ksi2, rho2), cross(ksi3, rho3), cross(ksi4, rho4), cross(ksi5, rho5), cross(ksi6, rho6)];
    J_omega = [ksi1, ksi2, ksi3, ksi4, ksi5, ksi6];

    J = [Jv; J_omega];

    %Get J_d so that we will get the relation between end's velocity and DH param d's velocity(fake).
    ksi1 = [0; 0; 1];
    ksi2 = T0_1(1:3, 3);
    ksi3 = T0_2(1:3, 3);
    ksi4 = T0_3(1:3, 3);
    ksi5 = T0_4(1:3, 3);
    ksi6 = T0_5(1:3, 3);

    J_d = [ksi1, ksi2, ksi3, ksi4, ksi5, ksi6];
    J_d = [J_d; zeros(3, 6)];

    %Get J_a. Note the choice of xi.
    ksi1 = T0_1(1:3, 1);
    ksi2 = T0_2(1:3, 1);
    ksi3 = T0_3(1:3, 1);
    ksi4 = T0_4(1:3, 1);
    ksi5 = T0_5(1:3, 1);
    ksi6 = T0_6(1:3, 1);

    J_a = [ksi1, ksi2, ksi3, ksi4, ksi5, ksi6];
    J_a = [J_a; zeros(3, 6)];

    %Get J_alpha. Only has 5 cols. Assume the z0 axis can not move. Note this. This may not be right.
    ksi1 = T0_1(1:3, 1);
    ksi2 = T0_2(1:3, 1);
    ksi3 = T0_3(1:3, 1);
    ksi4 = T0_4(1:3, 1);
    ksi5 = T0_5(1:3, 1);
    % ksi6 = [0;0;0];

    rho1 = T0_6(1:3, 4) - T0_1(1:3, 4);
    rho2 = T0_6(1:3, 4) - T0_2(1:3, 4);
    rho3 = T0_6(1:3, 4) - T0_3(1:3, 4);
    rho4 = T0_6(1:3, 4) - T0_4(1:3, 4);
    rho5 = T0_6(1:3, 4) - T0_5(1:3, 4);
    % rho6 = [0;0;0]; % NOTE this!

    Jv = [cross(ksi1, rho1), cross(ksi2, rho2), cross(ksi3, rho3), cross(ksi4, rho4), cross(ksi5, rho5)];
    J_omega = [ksi1, ksi2, ksi3, ksi4, ksi5];
    J_alpha = [Jv; J_omega];

    syms C; %C is the matrix for least square estimation of DH param's deviation
    J(1:3, :) = J(1:3, :) * 1e-3; % unit: m
    J_alpha(1:3, :) = J_alpha (1:3, :) * 1e-3; % unit: m

    C = [J, J_d, J_a, J_alpha];

    % C(1:3, :) = C(1:3, :) * 1e-3; % unit: m WRONG STEP! J_d AND J_a can not multiply 1e-3!
    C_fcn_handle = matlabFunction(C);

end
