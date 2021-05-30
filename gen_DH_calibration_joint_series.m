clc; clear;

%robot1
theta_series_bound = deg2rad([-100, -30; ...
                            -10, 30; ...
                            30, -30; ...
                            -20, 20; ...
                            -10, 60; ...
                            -10, -40]);
nn = 20;
joint_target_series = zeros(6, 3 * nn); % 18 sets of test joint pos.

for i = 1:6
    joint_target_series(i, :) = linspace(theta_series_bound(i, 1), theta_series_bound(i, 2), 3 * nn);
end

temp = joint_target_series';
save('robot1_DH_joint_series.txt', 'temp', '-double', '-ascii');

%robot2
theta_series_bound = deg2rad([90, 170; ...
                            - 20, 40; ...
                            40, -40; ...
                            -10, 20; ...
                            40, -20; ...
                            -50, 0]);
nn = 20;
joint_target_series = zeros(6, 3 * nn); % 18 sets of test joint pos.
theta123_target = zeros(3, nn);
for i = 1:3
    theta123_target(i, :) = linspace(theta_series_bound(i, 1), theta_series_bound(i, 2), nn);
end

for i = 1:3 % set theta1 2 and 3
    joint_target_series(1, i:3:end) = theta123_target(1, :);
    joint_target_series(2, i:3:end) = theta123_target(2, :);
    joint_target_series(3, i:3:end) = theta123_target(3, :);
end

for i = 1:3:3 * nn
    joint_target_series(4, i:i + 2) = linspace(theta_series_bound(4, 1), theta_series_bound(4, 2), 3);
    joint_target_series(5, i:i + 2) = linspace(theta_series_bound(5, 1), theta_series_bound(5, 2), 3);
    joint_target_series(6, i:i + 2) = linspace(theta_series_bound(6, 1), theta_series_bound(6, 2), 3);
end

temp = joint_target_series';
save('robot2_DH_joint_series.txt', 'temp', '-double', '-ascii');

%robot3
theta_series_bound = deg2rad([-60, 60; ...
                            0, 30; ...
                            -10, 30; ...
                            30, -30; ...
                            10, -40; ...
                            -10, 10]);
nn = 20;
joint_target_series = zeros(6, 3 * nn); % 18 sets of test joint pos.

for i = 1:6
    joint_target_series(i, :) = linspace(theta_series_bound(i, 1), theta_series_bound(i, 2), 3 * nn);
end

temp = joint_target_series';
save('robot3_DH_joint_series.txt', 'temp', '-double', '-ascii');
