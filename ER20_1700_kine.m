%% Problem2
clear; clc;
syms x1 x2 x3 x4 x5 x6; % x stands for theta.
syms L1 L2 L3 L4 L5 L8

% L1 = 162.5;
% L2 = 425;
% L3 = 392.2;
% L4 = 133.3;
% L5 = 99.7;
% L6 = 99.6;

T0_1 = eval(simplify(DH_T_2(x1, L1, L8, pi / 2)));
T1_2 = eval(simplify(DH_T_2(x2 - pi / 2, 0, -L2, 0)));
T2_3 = eval(simplify(DH_T_2(x3 + pi, 0, L3, pi / 2)));
T3_4 = eval(simplify(DH_T_2(x4, L4, 0, -pi / 2)));
T4_5 = eval(simplify(DH_T_2(x5, 0, 0, pi / 2)));
T5_6 = eval(simplify(DH_T_2(x6, L5, 0, 0)));

T0_6 = simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6)
f_T0_6 = matlabFunction(T0_6);

f_T0_1 = matlabFunction(T0_1);
T0_2 = simplify(T0_1 * T1_2);
f_T0_2 = matlabFunction(T0_2);
T0_3 = simplify(T0_2 * T2_3);
f_T0_3 = matlabFunction(T0_3);
T0_4 = simplify(T0_3 * T3_4);
f_T0_4 = matlabFunction(T0_4);
T0_5 = simplify(T0_4 * T4_5);
f_T0_5 = matlabFunction(T0_5);

syms J Jv J_omega ksi1 ksi2 ksi3 ksi4 ksi5 ksi6 rho1 rho2 rho3 rho4 rho5 rho6 % Get the Jaccobian mat of the robot. Choose the base frame 0 as the reference
syms J3_debug Jv_dbg Jomg_dbg rho1_dbg rho2_dbg rho3_dbg

ksi1 = [0; 0; 1];
ksi2 = T0_1(1:3, 3);
ksi3 = T0_2(1:3, 3);
ksi4 = T0_3(1:3, 3);
ksi5 = T0_4(1:3, 3);
ksi6 = T0_5(1:3, 3);

rho1 = simplify(T0_6(1:3, 4));
rho2 = simplify(T0_6(1:3, 4) - T0_1(1:3, 4));
rho3 = simplify(T0_6(1:3, 4) - T0_2(1:3, 4));
rho4 = simplify(T0_6(1:3, 4) - T0_3(1:3, 4));
rho5 = simplify(T0_6(1:3, 4) - T0_4(1:3, 4));
rho6 = [0; 0; 0]; % NOTE this!

Jv = [cross(ksi1, rho1), cross(ksi2, rho2), cross(ksi3, rho3), cross(ksi4, rho4), cross(ksi5, rho5), cross(ksi6, rho6)];
Jv = simplify(Jv)
J_omega = [ksi1, ksi2, ksi3, ksi4, ksi5, ksi6];
J_omega = simplify(J_omega)

J = simplify([Jv; J_omega])

rho1_dbg = T0_3(1:3, 4);
rho2_dbg = T0_3(1:3, 4) - T0_1(1:3, 4);
rho3_dbg = T0_3(1:3, 4) - T0_2(1:3, 4);

Jomg_dbg = [ksi1, ksi2, ksi3];
Jv_dbg = [cross(rho1_dbg, ksi1), cross(rho2_dbg, ksi2), cross(rho3_dbg, ksi3)];

J3_debug = simplify([Jv_dbg; Jomg_dbg])

syms J_d J_a J_alpha rho_temp ksi_temp % similar definition as the jacobian for joint position

% rho1 = simplify(T0_6(1:3,4));
% rho2 = simplify(T0_6(1:3,4) - T0_1(1:3,4));
% rho3 = simplify(T0_6(1:3,4) - T0_2(1:3,4));
% rho4 = simplify(T0_6(1:3,4) - T0_3(1:3,4));
% rho5 = simplify(T0_6(1:3,4) - T0_4(1:3,4));
% rho6 = [0;0;0]; % NOTE this!

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

rho1 = simplify(T0_6(1:3, 4) - T0_1(1:3, 4));
rho2 = simplify(T0_6(1:3, 4) - T0_2(1:3, 4));
rho3 = simplify(T0_6(1:3, 4) - T0_3(1:3, 4));
rho4 = simplify(T0_6(1:3, 4) - T0_4(1:3, 4));
rho5 = simplify(T0_6(1:3, 4) - T0_5(1:3, 4));
% rho6 = [0;0;0]; % NOTE this!

Jv = [cross(ksi1, rho1), cross(ksi2, rho2), cross(ksi3, rho3), cross(ksi4, rho4), cross(ksi5, rho5)];
Jv = simplify(Jv);
J_omega = [ksi1, ksi2, ksi3, ksi4, ksi5];
J_omega = simplify(J_omega);

J_alpha = simplify([Jv; J_omega]);

syms C; %C is the matrix for least square estimation of DH param's deviation
J(1:3, :) = J(1:3, :) * 1e-3; % unit: m
J_alpha(1:3, :) = J_alpha (1:3, :) * 1e-3; % unit: m

C = [J, J_d, J_a, J_alpha];

% C(1:3, :) = C(1:3, :) * 1e-3; % unit: m WRONG STEP! J_d AND J_a can not multiply 1e-3!
C_fcn = matlabFunction(C);

save('ER20_1700_kine_fcn.mat');
% N = 100;
% theta1 = linspace(0, pi, N)
% for i = 1:N
%
% end
