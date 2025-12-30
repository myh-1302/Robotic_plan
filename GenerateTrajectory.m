function [t_all, q_all, qd_all, qdd_all] = GenerateTrajectory(song_keys, song_beats, params)
    % GenerateTrajectory 生成木琴演奏的关节空间轨迹
    % 输入:
    %   song_keys: 琴键索引数组
    %   song_beats: 节拍数组
    %   params: 参数结构体，包含机器人限制、琴键位置等
    % 输出:
    %   t_all, q_all, qd_all, qdd_all: 轨迹数据

    % 解包参数
    dt = params.dt;
    beat_time = params.beat_time;
    q_start = params.q_start;
    keys_pos = params.keys_pos;
    R_ref = params.R_ref;
    wrist_center_x = params.wrist_center_x;
    lift_angle = params.lift_angle;
    lift_dir = params.lift_dir;
    z_hit = params.z_hit;
    
    % 新增参数解包
    if isfield(params, 'lift_angle_j3'), lift_angle_j3 = params.lift_angle_j3; else, lift_angle_j3 = 0; end
    if isfield(params, 'lift_angle_j2'), lift_angle_j2 = params.lift_angle_j2; else, lift_angle_j2 = 0; end
    if isfield(params, 'j4_yaw_factor'), j4_yaw_factor = params.j4_yaw_factor; else, j4_yaw_factor = 0; end
    if isfield(params, 'j6_roll_factor'), j6_roll_factor = params.j6_roll_factor; else, j6_roll_factor = 0; end
    if isfield(params, 'wrist_follow_factor'), wrist_follow_factor = params.wrist_follow_factor; else, wrist_follow_factor = 0; end
    
    v_max_travel = params.v_max_travel;
    a_max_travel = params.a_max_travel;
    v_max_strike = params.v_max_strike;
    a_max_strike = params.a_max_strike;

    % 初始化
    t_all = [];
    q_all = [];
    qd_all = [];
    qdd_all = [];
    last_q = q_start;
    
    % 预先规划手腕/基座的锚点 (Zone Planning)
    % 目标: 尽量保持 J1 稳定，只在必要时移动
    anchor_seq = PlanAnchors(song_keys, keys_pos, wrist_center_x, wrist_follow_factor);
    
    % 移动到第一个音符的准备位置
    first_key_idx = song_keys(1);
    pos_key_first = keys_pos(:, first_key_idx);
    
    % 计算第一个音符的目标姿态 (动态手腕中心)
    first_wrist_center_x = wrist_center_x + (pos_key_first(1) - wrist_center_x) * wrist_follow_factor;
    yaw_angle = atan2(pos_key_first(2), pos_key_first(1) - first_wrist_center_x);
    
    % 优化姿态 (J4/J6 搜索)
    % 传入 anchor_seq(1) 作为目标 J1
    [q_hit_first, R_hit_first] = OptimizeGrip(pos_key_first + [0; 0; z_hit], yaw_angle, R_ref, last_q, anchor_seq(1));
    
    % 计算第一个音符的悬停点 (动态分配 J2/J3)
    % 第一个音符没有"上一个琴键"，传入空或当前位置作为参考
    [d_j2, d_j3] = ComputeDynamicLift(pos_key_first, pos_key_first, lift_angle_j2, lift_angle_j3);
    q_hover = q_hit_first;
    q_hover(5) = q_hover(5) + lift_dir * lift_angle;
    q_hover(3) = q_hover(3) + lift_dir * d_j3;
    q_hover(2) = q_hover(2) + lift_dir * d_j2;
    
    % 规划到第一个悬停点
    % 降低起始速度，避免过快
    start_scale = 0.5;
    [t_seg, q_seg, qd_seg, qdd_seg] = TrapezoidalPlan(last_q, q_hover, v_max_travel * start_scale, a_max_travel * start_scale^2, dt);
    
    % 追加数据
    if ~isempty(t_all)
        t_seg = t_seg + t_all(end) + dt;
    end
    t_all = [t_all; t_seg];
    q_all = [q_all; q_seg];
    qd_all = [qd_all; qd_seg];
    qdd_all = [qdd_all; qdd_seg];
    last_q = q_hover;

    % 敲击第一个点
    [t_hit1, q_hit1, qd_hit1, qdd_hit1] = TrapezoidalPlan(last_q, q_hit_first, v_max_strike, a_max_strike, dt);
    if ~isempty(t_all), t_hit1 = t_hit1 + t_all(end) + dt; end
    t_all = [t_all; t_hit1];
    q_all = [q_all; q_hit1];
    qd_all = [qd_all; qd_hit1];
    qdd_all = [qdd_all; qdd_hit1];
    last_q = q_hit_first;
    
    % 循环生成后续音符的轨迹
    for i = 1:length(song_keys)-1
        % 当前已经是 Hit 状态 (last_q = q_hit_current)
        curr_key_idx = song_keys(i);
        next_key_idx = song_keys(i+1);
        
        pos_key_curr = keys_pos(:, curr_key_idx);
        pos_key_next = keys_pos(:, next_key_idx);
        
        % 1. 计算下一个击打点姿态
        % 下一个姿态 (动态手腕中心)
        next_wrist_center_x = wrist_center_x + (pos_key_next(1) - wrist_center_x) * wrist_follow_factor;
        yaw_angle_next = atan2(pos_key_next(2), pos_key_next(1) - next_wrist_center_x);
        pos_hit_next = pos_key_next + [0; 0; z_hit];
        
        % 使用 anchor_seq(i+1) 作为目标 J1
        [q_hit_next, R_hit_next] = OptimizeGrip(pos_hit_next, yaw_angle_next, R_ref, last_q, anchor_seq(i+1));
        
        % 2. 计算中间过渡点 (Bridge Point)
        % 位置: 两个琴键的中点，高度抬高
        % 高度策略: 基础抬高 + 距离越远抬得越高
        dist_keys = norm(pos_key_next - pos_key_curr);
        bridge_height = 30 + dist_keys * 0.12; % 稍微降低高度系数: 基础30mm + 12%距离
        pos_bridge = (pos_key_curr + pos_key_next) / 2;
        pos_bridge(3) = pos_bridge(3) + bridge_height;
        
        % 过渡点姿态: 
        % 关键修改: 参考姿态使用 (q_hit_curr + q_hit_next) / 2
        % 这样可以引导 IK 寻找一个在关节空间上介于起点和终点之间的解，避免 J2 突变
        q_ref_bridge = (last_q + q_hit_next) / 2;
        
        bridge_wrist_center_x = (wrist_center_x + next_wrist_center_x)/2; 
        yaw_angle_bridge = atan2(pos_bridge(2), pos_bridge(1) - bridge_wrist_center_x);
        
        % 过渡点的目标 J1 应该是两个锚点的平均值 (平滑过渡)
        target_j1_bridge = (anchor_seq(i) + anchor_seq(i+1)) / 2;
        [q_bridge, ~] = OptimizeGrip(pos_bridge, yaw_angle_bridge, R_ref, q_ref_bridge, target_j1_bridge);
        
        % 3. 规划路径: Hit_Curr -> Bridge -> Hit_Next
        
        % 限制 J1 J2 J3 速度 (大关节不宜过快)
        v_bridge = v_max_travel;
        v_bridge(1:3) = v_bridge(1:3) * 0.5; % 进一步降低大关节速度 (0.7 -> 0.5)
        a_bridge = a_max_travel;
        a_bridge(1:3) = a_bridge(1:3) * 0.5;

        % Segment 1: Hit -> Bridge (抬起并移动)
        [t_b1, q_b1, qd_b1, qdd_b1] = TrapezoidalPlan(last_q, q_bridge, v_bridge, a_bridge, dt);
        if ~isempty(t_all), t_b1 = t_b1 + t_all(end) + dt; end
        t_all = [t_all; t_b1];
        q_all = [q_all; q_b1];
        qd_all = [qd_all; qd_b1];
        qdd_all = [qdd_all; qdd_b1];
        
        % Segment 2: Bridge -> Hit_Next (下落并移动)
        [t_b2, q_b2, qd_b2, qdd_b2] = TrapezoidalPlan(q_bridge, q_hit_next, v_bridge, a_bridge, dt);
        if ~isempty(t_all), t_b2 = t_b2 + t_all(end) + dt; end
        t_all = [t_all; t_b2];
        q_all = [q_all; q_b2];
        qd_all = [qd_all; qd_b2];
        qdd_all = [qdd_all; qdd_b2];
        
        last_q = q_hit_next;
    end
    
    % 4. 演奏结束，回到初始姿态 (安全复位)
    % 先抬起
    q_end_hover = last_q;
    q_end_hover(5) = q_end_hover(5) + lift_dir * lift_angle;
    [t_end1, q_end1, qd_end1, qdd_end1] = TrapezoidalPlan(last_q, q_end_hover, v_max_travel, a_max_travel, dt);
    if ~isempty(t_all), t_end1 = t_end1 + t_all(end) + dt; end
    t_all = [t_all; t_end1];
    q_all = [q_all; q_end1];
    qd_all = [qd_all; qd_end1];
    qdd_all = [qdd_all; qdd_end1];
    last_q = q_end_hover;

    % 再回原点
    return_scale = 0.5; 
    [t_end, q_end, qd_end, qdd_end] = TrapezoidalPlan(last_q, q_start, v_max_travel * return_scale, a_max_travel * return_scale^2, dt);
    if ~isempty(t_all), t_end = t_end + t_all(end) + dt; end
    t_all = [t_all; t_end];
    q_all = [q_all; q_end];
    qd_all = [qd_all; qd_end];
    qdd_all = [qdd_all; qdd_end];
end

function [q_best, R_best] = OptimizeGrip(target_pos, base_yaw, R_ref, last_q, target_j1)
    % OptimizeGrip 
    % 目标: 最小化基座(J1)运动，同时保持整体平滑
    % 针对两侧琴键，增加 Yaw 搜索范围，允许手腕更多旋转
    
    % 搜索空间 (相对于 base_yaw 的偏移)
    % 扩大搜索范围: +/- 90度 (1.5 rad)
    yaw_offsets = linspace(-1.5, 1.5, 21); 
    
    % Roll 偏移 (影响 J6/手臂构型)
    roll_offsets = [-0.3, -0.15, 0, 0.15, 0.3];
    
    best_cost = inf;
    q_best = last_q;
    R_best = R_ref;
    
    % 增加对 J1 的软约束目标
    % 如果 target_pos Y 值很大，我们仍然希望 J1 接近 0
    % 但是物理上可能需要 J1 动一点。
    % 关键是: 我们希望找到一个解，其中 J1 尽可能小，而 J4/J5/J6 承担更多
    
    for dy = yaw_offsets
        for dr = roll_offsets
            % 构建试探姿态
            yaw_curr = base_yaw + dy;
            roll_curr = dr;
            
            R_z = [cos(yaw_curr) -sin(yaw_curr) 0; sin(yaw_curr) cos(yaw_curr) 0; 0 0 1];
            R_x = [1 0 0; 0 cos(roll_curr) -sin(roll_curr); 0 sin(roll_curr) cos(roll_curr)];
            R_curr = R_z * R_x * R_ref;
            
            % 求解 IK
            q_curr = GetStickIK(target_pos, R_curr, last_q);
            
            % 如果无解 (NaN)，跳过
            if any(isnan(q_curr))
                continue;
            end

            % 计算代价函数
            % 1. 基座惩罚: 希望 J1 接近 target_j1 (锚点)
            % 权重极大，强制优先使用手腕
            cost_base = abs(q_curr(1) - target_j1);
            
            % 2. 平滑惩罚: 希望关节变化小
            % 加权: J2(idx 2) 权重加大，防止肩部剧烈抖动
            diff = abs(q_curr - last_q);
            % J1:1, J2:5, J3:2, J4-J6:0.5
            % cost_smooth = dot(diff, [1, 5, 2, 0.5, 0.5, 0.5]); 
            cost_smooth = dot(diff, [100, 100, 100, 0.1, 0.1, 0.1]);
            
            % 3. 舒适姿态惩罚 (可选): 避免关节极限
            
            % 组合代价
            % 提高 J1 惩罚权重 (20.0 -> 50.0)
            total_cost = 1000.0 * cost_base + 1.0 * cost_smooth;
            
            if total_cost < best_cost
                best_cost = total_cost;
                q_best = q_curr;
                R_best = R_curr;
            end
        end
    end
end

function anchor_seq = PlanAnchors(song_keys, keys_pos, wrist_center_x, wrist_follow_factor)
    % PlanAnchors 预先规划每个音符的 J1 锚点
    % 策略: 
    %   1. 遍历所有音符，计算其理想的 J1 角度 (yaw)
    %   2. 尝试将连续的音符分组 (Zone)，在同一组内使用相同的 J1 锚点
    %   3. 如果一组内的音符跨度超过手腕的舒适范围，则开启新的组
    
    num_notes = length(song_keys);
    ideal_yaws = zeros(1, num_notes);
    
    % 1. 计算所有音符的理想 Yaw
    for i = 1:num_notes
        pos_key = keys_pos(:, song_keys(i));
        % 使用与 GenerateTrajectory 相同的逻辑计算理想 Yaw
        curr_wrist_center_x = wrist_center_x + (pos_key(1) - wrist_center_x) * wrist_follow_factor;
        ideal_yaws(i) = atan2(pos_key(2), pos_key(1) - curr_wrist_center_x);
    end
    
    anchor_seq = zeros(1, num_notes);
    
    % 2. 分组规划
    % 手腕舒适偏航范围 (单侧)
    % 假设手腕可以轻松偏转 +/- 20度 (0.35 rad)
    max_wrist_dev = deg2rad(180); 
    
    i = 1;
    while i <= num_notes
        % 尝试从 i 开始建立一个窗口
        window_start = i;
        window_end = i;
        
        % 贪婪扩展窗口
        while window_end < num_notes
            next_idx = window_end + 1;
            
            % 检查加入下一个音符后，窗口内的跨度是否过大
            current_window_yaws = ideal_yaws(window_start:next_idx);
            span = max(current_window_yaws) - min(current_window_yaws);
            
            % 如果跨度超过 2 * max_wrist_dev，说明无法用一个中心覆盖
            if span > 2 * max_wrist_dev
                break;
            end
            
            window_end = next_idx;
        end
        
        % 确定当前窗口的锚点
        % 取窗口内最大最小值的中间，这样覆盖最均匀
        window_yaws = ideal_yaws(window_start:window_end);
        anchor_val = (min(window_yaws) + max(window_yaws)) / 2;
        
        % 赋值给序列
        anchor_seq(window_start:window_end) = anchor_val;
        
        % 移动指针
        i = window_end + 1;
    end
end

function [delta_j2, delta_j3] = ComputeDynamicLift(target_key_pos, current_key_pos, max_lift_j2, max_lift_j3)
    delta_j2 = 0;
    delta_j3 = 0;
end
