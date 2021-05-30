clear; clc;
load('ER20_1700_kine_fcn.mat');

L1 = 504;
L2 = 780;
L3 = 140;
L4 = 760;
L5 = 125;
L8 = 170;

th = [(pi*3.0 / 4.0), 0.5, -0.3, 0, -0.3, 0];
x1 = th(1);
x2 = th(2);
x3 = th(3);
x4 = th(4);
x5 = th(5);

J_fcn = matlabFunction(J)

det(J_fcn(L2,L3,L4,L5,L8,x1,x2,x3,x4,x5))