clear; clc;

load('ER20_1700_kine_fcn.mat');

L1 = 504;
L2 = 780;
L3 = 140;
L4 = 760;
L5 = 125;
L8 = 170;

t_0 = 0;
t_f = 10;
dt = 0.05;
t = t_0: dt: t_f;

X0 = deg2rad([0, 0, 0, 0, 0, 0]);

kn_T0_6 = f_T0_6(L1, L2, L3, L4, L5, L8, X0(1), X0(2), X0(3), X0(4), X0(5), X0(6));
% LINE TRAJ
pose0 = kn_T0_6(1:3, 1:3);
position0 = kn_T0_6(1:3, 4);

pose1 = pose0;
translation = [0; 300; 0];
position1 = position0 + translation;

% Pose planning
Psi_0 = deg2rad([180, 90, 0]');
Psi_f = deg2rad([180, 90, 90]'); % initial pose when all joints' pos is 0;

Re_0 = XYZ_Euler(Psi_0(1), Psi_0(2), Psi_0(3)); % Initial R of the end frame.
Re_f = XYZ_Euler(Psi_f(1), Psi_f(2), Psi_f(3)); % End R of the end frame.

delta_R = transpose(Re_0)*Re_f;
phi = acos((trace(delta_R)-1) / 2); %Equivalent rotation angle about axis k from Re_0 to Re_f .
k_axis = ( 1/(2*sin(phi)) )*[delta_R(3,2)-delta_R(2,3);...
delta_R(1,3)-delta_R(3,1); delta_R(2,1)-delta_R(1,2)];

Psi_t = zeros(3,length(t)); %XYZ_euler angle of end frame. There are 2 solutions.
d_Psi_t = zeros(3,length(t)); %Angular Velocity of XYZ_euler angle.
Omega_t = zeros(3,length(t)); %Angular Velocity of end frmae.

X_plan = zeros(8, 6, length(t));

for i = 1:length(t)
    tau = (t(i)-t_0)/(t_f-t_0);
    lambda = 10*tau^3 - 15*tau^4 + 6*tau^5; % Zero initial condition for v and acc.
    d_lambda = (1/(t_f-t_0)) * (30*tau^2- 60*tau^3 + 30*tau^4);%dlambda/dt

    position_plan = position0 + lambda*(position1-position0);

    D_r = Rot_K(k_axis, lambda*phi); %Drive matrix at t(i)
    
    % disable orientation planning

     R_ti = Re_0*D_r; %R of the end frame at t(i).
     plan_T0_6 = [R_ti, position_plan; 0, 0, 0, 1]
    % plan_T0_6 = [Re_0, position_plan; 0, 0, 0, 1]

    X_temp = efort_inv2(plan_T0_6, L1, L2, L3, L4, L5, L8, f_T0_3, f_T0_4)

    if (size(X_temp, 1) < 8) % Sometimes the function efort_inv2 can not yeild 8 solutions
        X_plan(1:size(X_temp, 1), :, i) = X_temp;
    else
        X_plan(:, :, i) = X_temp;
    end
    % Psi_t(:, :, i) = Tr2Xyz_New_my(R_ti);
    % Q_t(:,:, i) = ikine_3DOF_Wrist_fcn(R_ti);
    % Omega_t(:, i) = Re_0*k_axis*phi*d_lambda;

end

% Psi_t_choice = reshape(Psi_t(1,:,:), [3, length(t)]); %Choose one solution between two.





% for i = 1:100
%     lambda = lambda + delta;
%     position_plan = (position1 - position0) * lambda + position0;
%     plan_T0_6 = [pose1, position_plan; 0, 0, 0, 1];

%     X_temp = efort_inv(plan_T0_6, L1, L2, L3, L4, L5, L8, f_T0_3, f_T0_4);

%     if (size(X_temp, 1) < 8)
%         X_plan(1:size(X_temp, 1), :, i) = X_temp;
%     else
%         X_plan(:, :, i) = X_temp;
%     end

% end

X1_plan = zeros(6, length(t));

for i = 1:length(t)

    if (i == 1)
        [~, idx] = min(sum(abs(X_plan(:, :, 1) - X0), 2)); % sum(X,2), sum along each row's elements
        X1_plan(:, 1) = transpose(X_plan(idx, :, 1));
    else
         [~, idx] = min(sum(abs(X_plan(:, :, i) - transpose(X1_plan(:, i-1)) ), 2)); % sum(X,2), sum along each row's elements
         X1_plan(:, i) = transpose(X_plan(idx, :, i));
    end
    
end

% X1_plan = reshape(X_plan(1, :, :), 6, 100);

figure(1)
hold on

for i = 1:6
    plot(X1_plan(i, :),'-x');
end

legend
