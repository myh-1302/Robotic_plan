function theta_solutions = Ikine6s(T_target, config)
    % Ikine6s 求解6自由度机器人的逆运动学
    % 输入:
    %   T_target - 末端执行器的目标变换矩阵(4x4)
    %   config   - 机器人结构参数(包括L1, L2, L3)
    % 输出:
    %   theta_solutions - 6个关节角度(2x6矩阵，两组解)
    
    % 获取机器人结构参数
    L1 = config.L1;
    L2 = config.L2;
    L3 = config.L3;
   
    
    % 定义机器人螺旋轴参数(结构固定参数)
    w0 = [0,0,0,0,0,0; 0,1,1,0,1,0; 1,0,0,1,0,1];  % 角速度方向(3x6)
    q0 = [0,0,0,0,0,0; 0,0,0,0,0,0; 0,491,941,1391,1391,1391];  % 位置向量(3x6)
    T0 = [-1,0,0,0; 0,-1,0,0; 0,0,1,1391; 0,0,0,1];  % 初始变换矩阵
    
    % 构建螺旋轴(6x6，每列对应一个关节的参数)
    x0 = zeros(6,6);
    for i = 1:6
        w_i = w0(1:3,i);
        q_i = q0(1:3,i);
        x0(1:3,i) = -cross(w_i, q_i);  % 线速度方向
        x0(4:6,i) = w_i;               % 角速度方向
    end
    
    % 计算相对变换矩阵 G = T_target / T0 = T_target * inv(T0)
    G = T_target / T0;
    
    % 计算参考点在G坐标系下的位置
    pw = [0; 0; L1 + L2 + L3; 1];  % 参考点
    q = G * pw;                    % 变换后的参考点(q(1:3)为x,y,z)
    
    % 求解theta1
 
    t11 = atan2(q(2),q(1)); 
    % 求解theta2和theta3
    [t21,t31] = cal_theta23(q, L1, L2, L3);
    

    
    % 初始化结果矩阵（稍后会被重新初始化为2x6）
    theta_temp = zeros(1,6);
    theta_temp(1:3)=[t11,t21,t31];
    
    % 计算前三个关节的变换矩阵 T_03（相对于基座，不包含T0）
    % 因为 G = T_target / T0 = T1*T2*T3*T4*T5*T6
    % 所以 T_03 应该是 T1*T2*T3（相对于基座）
    T_03 = eye(4);
    for i = 1:3
        T_03 = T_03 * exp_rot(x0(:, i), theta_temp(i));
    end
    
    % 提取目标旋转矩阵（G的旋转部分）
    R_target = G(1:3, 1:3);
    R_03 = T_03(1:3, 1:3);
    
    % 计算后三个关节需要实现的旋转矩阵 R_36
    % R_target = R_03 * R_36，所以 R_36 = R_03' * R_target
    R_36 = R_03' * R_target;
    
    % 将 R_36 转换为 ZYZ 欧拉角 (theta4, theta5, theta6)，返回两组解
    [t4_sol, t5_sol, t6_sol] = rotm2zyz(R_36);
    
    % 初始化两组解
    theta_solutions = zeros(1, 6);
    theta_solutions(1:3) = [t11, t21, t31];
    
    % 第一组解
    theta_solutions(4:6) = [t4_sol(1), t5_sol(1), t6_sol(1)];
    % % 第二组解
    % theta_solutions(2, 4:6) = [t4_sol(2), t5_sol(2), t6_sol(2)];

    % 内部函数：计算theta2和theta3的角度
    function [t21, t31] = cal_theta23(q, L1, L2, L3)
        v = q(1)^2 + q(2)^2 + (q(3) - L1)^2;  % 距离平方
        sqrt_v = sqrt(v);
        
        % 检查工作空间范围
        if (sqrt_v + L3 > L2) && (abs(sqrt_v - L3) < L2)
            n = v + L2^2 - L3^2;
            n1 = L2^2 + L3^2 - v;
            t2 = acos((q(3) - L1) / sqrt_v);
            o2 = acos(n / (2 * L2 * sqrt_v));
            o3 = acos(n1 / (2 * L2 * L3));
            t21 = t2 - o2;
            
            t31 = pi - o3;
    
        elseif abs(sqrt_v + L3 - L2) < 1e-6  % 边界奇异情况
            t31 = pi;
      
            t2 = acos((q(3) - L1) / sqrt_v);
            t21 = t2;
       
        else
            disp("目标位置超出工作空间，无法求解");
            t21 = NaN;
        
            t31 = NaN;
          
        end
    end

    % 内部函数：计算旋转关节的旋转矩阵(指数映射)
    function T = exp_rot(x, t)
        w = x(4:6);  % 旋转轴
        v = x(1:3);
        if norm(w) < eps
            R = eye(3);  % 移动关节(此处为旋转关节，理论上不会进入)
        else
            w_norm = norm(w);
            w_unit = w / w_norm;
            w_hat = skew(w_unit);  % 斜对称矩阵
            % 罗德里格斯公式
            R = eye(3) + sin(w_norm * t) * w_hat + (1 - cos(w_norm * t)) * (w_hat * w_hat);
            p = (eye(3) - R) * (cross(w_unit, v / w_norm)) + ...
            (w_unit * w_unit' * v * t);
        end
        T =[R,p;0,0,0,1];
    end

    % 内部函数：将旋转矩阵转换为ZYZ欧拉角，返回两组解
    function [phi, theta, psi] = rotm2zyz(R)
        % 将旋转矩阵转换为ZYZ欧拉角
        % R = Rz(phi) * Ry(theta) * Rz(psi)
        % phi = theta4, theta = theta5, psi = theta6
        % 返回两组解
        %
        % 旋转矩阵元素：
        % R(3,3) = cos(theta)
        % R(1,3) = cos(phi)*sin(theta)
        % R(2,3) = sin(phi)*sin(theta)
        % R(3,1) = -sin(theta)*cos(psi)
        % R(3,2) = sin(theta)*sin(psi)
        
        % 检查奇异情况
        if abs(abs(R(3,3)) - 1) < 1e-6
            % 奇异情况：theta5 = 0 或 pi
            if R(3,3) > 0
                % theta5 = 0，此时 phi 和 psi 不能唯一确定
                % 设 phi = 0，则 psi = atan2(R(2,1), R(1,1))
                theta = [0; 0];
                phi = [0; 0];
                psi = [atan2(R(2,1), R(1,1)); atan2(R(2,1), R(1,1))];
            else
                % theta5 = pi，此时 phi 和 psi 不能唯一确定
                % 设 phi = 0，则 psi = atan2(-R(2,1), -R(1,1))
                theta = [pi; pi];
                phi = [0; 0];
                psi = [atan2(-R(2,1), -R(1,1)); atan2(-R(2,1), -R(1,1))];
            end
        else
            % 正常情况：有两组解
            % 第一组解：theta5 = acos(R(3,3))，范围 [0, pi]
            theta1 = acos(R(3,3));
            sin_theta1 = sin(theta1);
            
            % 提取第一组解
            phi1 = atan2(R(2,3), R(1,3));
            psi1 = atan2(R(3,2), -R(3,1));
            
            % 第二组解：theta5 = -theta1
            % 由于 sin(-theta) = -sin(theta)，需要调整phi和psi
            theta2 = -theta1;
            
            % 对于第二组解：
            % R(1,3) = cos(phi1)*sin(theta1) = cos(phi2)*sin(theta2) = -cos(phi2)*sin(theta1)
            % 所以 cos(phi1) = -cos(phi2)，sin(phi1) = -sin(phi2)
            % 因此 phi2 = phi1 + pi
            phi2 = phi1 + pi;
            
            % 对于psi：
            % R(3,1) = -sin(theta1)*cos(psi1) = -sin(theta2)*cos(psi2) = sin(theta1)*cos(psi2)
            % R(3,2) = sin(theta1)*sin(psi1) = sin(theta2)*sin(psi2) = -sin(theta1)*sin(psi2)
            % 所以 cos(psi1) = -cos(psi2)，sin(psi1) = -sin(psi2)
            % 因此 psi2 = psi1 + pi
            psi2 = psi1 + pi;
            
            % 将角度归一化到 [-pi, pi] 范围
            phi2 = atan2(sin(phi2), cos(phi2));
            psi2 = atan2(sin(psi2), cos(psi2));
            
            phi = [phi1; phi2];
            theta = [theta1; theta2];
            psi = [psi1; psi2];
        end
    end

    % 内部函数：斜对称矩阵转换
    function S = skew(w)
        S = [0, -w(3), w(2);
             w(3), 0, -w(1);
             -w(2), w(1), 0];
    end

end
