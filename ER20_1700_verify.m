clc; clear;
load 'ER20_1700_kine_fcn.mat';

X = deg2rad([0,0, 0, 0, 0, 0]);
L1 = 504;
L2 = 780;
L3 = 140;
L4 = 760;
L5 = 125;
L8 = 170;

x1 = X(1);
x2 = X(2);
x3 = X(3);
x4 = X(4);
x5 = X(5);
x6 = X(6);

for i = 1:8
    temp1 = f_T0_1(L1,L8,x1);
    temp2 = f_T0_2(L1,L2,L8,x1,x2);
    temp3 = f_T0_3(L1,L2,L3,L8,x1,x2,x3);
    temp4 = f_T0_4(L1,L2,L3,L4,L8,x1,x2,x3,x4);
    temp5 = f_T0_5(L1,L2,L3,L4,L8,x1,x2,x3,x4,x5);
    temp6 = f_T0_6(L1,L2,L3,L4,L5,L8,x1,x2,x3,x4,x5,x6);
    pos_o1 = temp1(:,4);
    pos_o2 = temp2(:,4);
    pos_o3 = temp3(:,4);
    pos_o4 = temp4(:,4);
    pos_o5 = temp5(:,4);
    pos_o6 = temp6(:,4);
    M = [pos_o1, pos_o2,pos_o3,pos_o4,pos_o5,pos_o6];
    pos_o0 = zeros(4,1);
    M=[pos_o0,M];

    plot3(M(1,:), M(2,:), M(3,:), '-x')
    axis equal
    hold on
    
end