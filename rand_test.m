clear; clc;

load('ER20_1700_kine_fcn.mat');

L1 = 504;
L2 = 780;
L3 = 140;
L4 = 760;
L5 = 125;
L8 = 170;

N = 100;

X = [0,1,0.4,0,0,0]
kn_T0_6 = f_T0_6(L1, L2, L3, L4, L5, L8, X(1), X(2), X(3), X(4), X(5), X(6))
X_verify = efort_inv2(kn_T0_6, L1, L2, L3, L4, L5, L8, f_T0_3, f_T0_4)


for i = 1:N
    sprintf('i = %d\n', i)
    X = deg2rad(-90 + 180*rand(1,6));
    disp(X)
    kn_T0_6 = f_T0_6(L1, L2, L3, L4, L5, L8, X(1), X(2), X(3), X(4), X(5), X(6));
    X_verify = efort_inv2(kn_T0_6, L1, L2, L3, L4, L5, L8, f_T0_3, f_T0_4)
%     X_verify = rad2deg(X_verify)
end
