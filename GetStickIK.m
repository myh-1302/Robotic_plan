function q_sol = GetStickIK(target_pos, target_rot, last_q)
    % GetStickIK 计算给定棍子尖端位置和法兰姿态的关节角度
    % 输入:
    %   target_pos: 棍子尖端的3x1向量(x, y, z)
    %   target_rot: 法兰(J6坐标系)的3x3旋转矩阵
    %   last_q: 上一时刻的6个关节角度(用于解的选择)
    % 输出:
    %   q_sol: 6个关节角度的向量

    % 机器人配置参数
    config.L1 = 491;
    config.L2 = 450;
    config.L3 = 450;
    
    % J6坐标系下的棍子偏移量 (来自 GetJ6Point.m)
    p_j6_xyz = [177.51; 0; 13];
    
    % 计算法兰位置
    % P_stick = P_flange + R_flange * p_j6_xyz
    % P_flange = P_stick - R_flange * p_j6_xyz
    p_flange = target_pos - target_rot * p_j6_xyz;
    
    % 构建法兰的目标变换矩阵
    T_target_flange = eye(4);
    T_target_flange(1:3, 1:3) = target_rot;
    T_target_flange(1:3, 4) = p_flange;
    
    % 调用逆运动学
    % 注意: Ikine6s 返回 2x6 矩阵
    theta_solutions = Ikine6s(T_target_flange, config);
    
    % 选择最优解
    % 过滤掉复数解
    valid_indices = [];
    for i = 1:size(theta_solutions, 1)
        if isreal(theta_solutions(i, :))
            valid_indices = [valid_indices, i];
        end
    end
    
    if isempty(valid_indices)
        % warning('目标位置无实数逆解。');
        q_sol = nan(1, 6); % 返回 NaN 表示无解
        return;
    end
    
    best_idx = -1;
    min_dist = inf;
    
    for i = valid_indices
        q_curr = theta_solutions(i, :);
        
        % 处理关节限制或角度缠绕 (例如 -pi 到 pi)
        % 假设 Ikine6s 返回归一化的角度。
        % 我们应该检查 2*pi 的跳变以保证运动平滑。
        
        % 简单的欧氏距离
        dist = norm(q_curr - last_q);
        
        % 检查 2*pi 的多解情况
        % 如果 q_curr 是 -3.1 而 last_q 是 3.1，距离很大但实际移动很小。
        % 假设 Ikine6s 返回值在 [-pi, pi] 之间。
        % 我们可以通过加减 2*pi*k 来调整 q_curr 使其接近 last_q。
        
        q_curr_adjusted = q_curr;
        for j = 1:6
            diff = q_curr_adjusted(j) - last_q(j);
            while diff > pi
                q_curr_adjusted(j) = q_curr_adjusted(j) - 2*pi;
                diff = q_curr_adjusted(j) - last_q(j);
            end
            while diff < -pi
                q_curr_adjusted(j) = q_curr_adjusted(j) + 2*pi;
                diff = q_curr_adjusted(j) - last_q(j);
            end
        end
        
        dist = norm(q_curr_adjusted - last_q);
        
        if dist < min_dist
            min_dist = dist;
            best_idx = i;
            q_sol = q_curr_adjusted;
        end
    end
end
