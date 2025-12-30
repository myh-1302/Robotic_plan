% main.m
% 机器人木琴演奏主程序
% 机器人: ZYYZYZ 6自由度
% 功能: 选择曲目，生成轨迹，可视化机械臂运动，导出数据

clear; clc; close all;

%% 1. 参数设置 
% ---------------------------------------------------------
song_name = 'TwoTigers'; % 可选: 'LittleStar', 'TwoTigers', 'OdeToJoy', 'TurkishMarch', 'FlowerSea', 'TheWindRises', 'JasmineFlower'
tempo = 3;              % 速度倍率 (越大越快)
dt = 0.001;               % 时间步长 (秒)
% ---------------------------------------------------------
% 木琴琴键定义
key_y = [-177, -152, -127, -100, -75, -50, -25, 0, 27, 53, 80, 105, 128, 155, 180];
key_x = 670.8736;
key_z = 460;
num_keys = length(key_y);
% 机器人运动学参数
% 关节限制 / 基准速度 / 加速度 (Rad/s, Rad/s^2)
v_base_travel = deg2rad(120);
a_base_travel = deg2rad(300);
v_base_strike = deg2rad(300);
a_base_strike = deg2rad(1000);

% 根据 tempo 调整限制
params.v_max_travel = v_base_travel * tempo * ones(1, 6);
params.a_max_travel = a_base_travel * tempo^2 * ones(1, 6);
params.v_max_strike = v_base_strike * tempo * ones(1, 6);
params.a_max_strike = a_base_strike * tempo^2 * ones(1, 6);

params.dt = dt;
params.beat_time = 0.6 / tempo;

% 起始姿态
params.q_start = [-3.395, 25.349, 108.201, -2.878, 55, -9.904] * pi/180;

% 获取初始姿态 (参考)
T_start = GetJ6Point(params.q_start);
params.R_ref = T_start(1:3, 1:3);

% 优化策略参数
% 棍子长度约 177.51mm
params.wrist_center_x = key_x - 177.51; 
params.lift_angle = deg2rad(10); 
params.lift_angle_j3 = deg2rad(0); % J3 抬起角度 
params.lift_dir = -1; % J5减小为抬起
params.j4_yaw_factor = 0.2; % J4 参与移动的比例系数 (0~1)
params.j6_roll_factor = 0.8; % J6 参与姿态调整的比例系数 (与J4配合)
params.wrist_follow_factor = 0; % 虚拟手腕中心跟随琴键移动的比例 


params.keys_pos = zeros(3, num_keys);
for i = 1:num_keys
    params.keys_pos(:, i) = [key_x; key_y(i); key_z];
end
params.z_hit = -4; % 敲击深度

%% 2. 获取曲目与生成轨迹
disp(['正在生成曲目: ' song_name ' ...']);
[song_keys, song_beats] = GetSong(song_name);
[t_all, q_all, qd_all, qdd_all] = GenerateTrajectory(song_keys, song_beats, params);

disp(['轨迹生成完成。总时长: ' num2str(t_all(end)) ' 秒']);

%% 3. 可视化 (机械臂末端/法兰轨迹)
disp('正在绘制可视化...');

num_points = length(t_all);
flange_pos = zeros(num_points, 3);
stick_pos = zeros(num_points, 3);

for i = 1:num_points
    % 计算法兰位置 (机械臂末端)
    T_f = GetRobotFK(q_all(i, :));
    flange_pos(i, :) = T_f(1:3, 4)';
    
    % 计算棍子尖端位置 (用于对比)
    T_s = GetJ6Point(q_all(i, :));
    stick_pos(i, :) = T_s(1:3, 4)';
end

% 图 1: 3D 路径对比
figure('Name', '机械臂与塑料棍轨迹');
plot3(flange_pos(:,1), flange_pos(:,2), flange_pos(:,3), 'b', 'LineWidth', 1.5); hold on;
plot3(stick_pos(:,1), stick_pos(:,2), stick_pos(:,3), 'g--', 'LineWidth', 1);
plot3(params.keys_pos(1,:), params.keys_pos(2,:), params.keys_pos(3,:), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
grid on; axis equal;
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
legend('机械臂轨迹', '末端轨迹', '琴键位置');
title(['机械臂运动轨迹 - ' song_name]);

% 图 2: 关节空间
figure('Name', '关节空间数据');
subplot(3,1,1); plot(t_all, q_all); title('关节位置 (deg)'); grid on; legend('J1','J2','J3','J4','J5','J6');
subplot(3,1,2); plot(t_all, qd_all); title('关节速度 (deg/s)'); grid on;
subplot(3,1,3); plot(t_all, qdd_all); title('关节加速度 (deg/s^2)'); grid on;

% 图 3: 笛卡尔空间 (法兰)
figure('Name', '机械臂的笛卡尔数据');
subplot(3,1,1); plot(t_all, flange_pos); title('位置 (mm)'); legend('X','Y','Z'); grid on;
% 简单差分计算速度
flange_vel = [zeros(1,3); diff(flange_pos)/dt];
subplot(3,1,2); plot(t_all, flange_vel); title('速度 (mm/s)'); grid on;

%% 4. 输出生成
output_data = rad2deg(q_all);
file_name = 'data_myh4.txt';
fid = fopen(file_name, 'w');
format_str = '%.4f %.4f %.4f %.4f %.4f %.4f\n';
for i = 1:size(output_data, 1)
    fprintf(fid, format_str, output_data(i, :));
end
fclose(fid);

disp(['数据已保存至 ' file_name]);
t = t_all;
q = q_all;
qd = qd_all;
qdd = qdd_all;

