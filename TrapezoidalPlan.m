function [t, q, qd, qdd] = TrapezoidalPlan(q_start, q_end, v_max, a_max, dt)
    % TrapezoidalPlan 生成多关节同步的梯形速度规划轨迹
    % 输入:
    %   q_start: 1xN 起始位置
    %   q_end: 1xN 终止位置
    %   v_max: 1xN 或标量 最大速度
    %   a_max: 1xN 或标量 最大加速度
    %   dt: 时间步长
    % 输出:
    %   t: 时间向量
    %   q: 位置矩阵 (MxN)
    %   qd: 速度矩阵 (MxN)
    %   qdd: 加速度矩阵 (MxN)

    num_joints = length(q_start);
    if length(v_max) == 1, v_max = repmat(v_max, 1, num_joints); end
    if length(a_max) == 1, a_max = repmat(a_max, 1, num_joints); end

    dist = q_end - q_start;
    
    % 计算每个关节的最小运动时间
    T_min = zeros(1, num_joints);
    for i = 1:num_joints
        D = abs(dist(i));
        V = v_max(i);
        A = a_max(i);
        
        if D < 1e-6
            T_min(i) = 0;
            continue;
        end
        
        % 检查三角形速度曲线是否足够（未达到峰值速度）
        % D = V_peak^2 / A. 如果 V_peak < V, 则 T = 2 * sqrt(D/A)
        if D * A <= V^2
            T_min(i) = 2 * sqrt(D / A);
        else
            % 梯形速度曲线
            % T = D/V + V/A
            T_min(i) = D / V + V / A;
        end
    end
    
    % 同步：使用最长的时间
    T_total = max(T_min);
    
    % 向上取整到最近的 dt 整数倍
    T_total = ceil(T_total / dt) * dt;
    
    if T_total == 0
        t = 0;
        q = q_start;
        qd = zeros(1, num_joints);
        qdd = zeros(1, num_joints);
        return;
    end
    
    num_steps = round(T_total / dt) + 1;
    t = (0:num_steps-1)' * dt;
    q = zeros(num_steps, num_joints);
    qd = zeros(num_steps, num_joints);
    qdd = zeros(num_steps, num_joints);
    
    for i = 1:num_joints
        D = dist(i);
        sign_D = sign(D);
        D = abs(D);
        
        if D < 1e-6
            q(:, i) = q_start(i);
            continue;
        end
        
        % 重新计算该关节的 V 和 A 以匹配 T_total
        % 我们有自由度选择 V 和 A。
        % 策略：保持 A 不变（如果可能），降低 V。
        % T = D/V + V/A. 关于 V 的二次方程。
        % V^2/A - T*V + D = 0.
        % V = (T - sqrt(T^2 - 4*D/A)) * A / 2. (取较小的根以满足约束)
        
        % 我们需要 T_total^2 >= 4*D/A.
        % 如果不满足，我们需要降低 A。
        % 4*D/A <= T_total^2 => A >= 4*D/T_total^2.
        % 所以如果 a_max(i) 满足此条件，我们可以保持 A = a_max(i)。
        
        A = a_max(i);
        if A < 4 * D / T_total^2
             % 如果 T_total >= T_min 计算正确，这种情况不应发生
             % 但为了保险起见，设置 A 为极限值
             A = 4 * D / T_total^2;
        end
        
        % 计算加速时间 Ta
        discriminant = T_total^2 - 4 * D / A;
        if discriminant < 0
            discriminant = 0;
        end
        Ta = (T_total - sqrt(discriminant)) / 2;
        V_cruise = A * Ta;
        
        % 生成曲线
        for k = 1:num_steps
            time = t(k);
            if time < Ta
                % 加速段
                q(k, i) = 0.5 * A * time^2;
                qd(k, i) = A * time;
                qdd(k, i) = A;
            elseif time < T_total - Ta
                % 匀速段
                q(k, i) = 0.5 * A * Ta^2 + V_cruise * (time - Ta);
                qd(k, i) = V_cruise;
                qdd(k, i) = 0;
            else
                % 减速段
                t_decel = time - (T_total - Ta);
                q(k, i) = 0.5 * A * Ta^2 + V_cruise * (T_total - 2*Ta) + ...
                          V_cruise * t_decel - 0.5 * A * t_decel^2;
                qd(k, i) = V_cruise - A * t_decel;
                qdd(k, i) = -A;
            end
        end
        
        % 应用方向和起始位置
        q(:, i) = q_start(i) + sign_D * q(:, i);
        qd(:, i) = sign_D * qd(:, i);
        qdd(:, i) = sign_D * qdd(:, i);
    end
end
