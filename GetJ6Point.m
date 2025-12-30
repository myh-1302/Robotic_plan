function p_base = GetJ6Point(theta)
    % GetJ6Point 计算敲击的位置
    % 输入:
    %   theta - 6个关节角度(1x6向量，单位：rad)
    % 输出:
    %   p_base - 点在基坐标系下的位置(3x1向量，单位：mm)
    %
    % 说明:
    %   J6轴是第6个关节轴，计算前6个关节确定J6的位置和姿态
    

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
    
    % 计算前6个关节的变换矩阵 T_06
    T_06 = eye(4);
    for i = 1:6
        T_06 = T_06 * exp_w(x0(:, i), theta(i));
    end
    T_06 = T_06 * T0;
    
    % 
    p_j6 = [177.51; 0;13; 1];
    
    % 转换到基坐标系
    p_base_homogeneous = T_06 * p_j6;
    T_06(:,4) = p_base_homogeneous;
    p_base = T_06;

    
    % 内部函数：计算旋转关节的旋转矩阵(指数映射)
    function exp_m = exp_w(x, t)
        v = x(1:3);  % 线速度参数
        w = x(4:6);  % 角速度参数

        if norm(w) < eps
            % 处理移动关节
            R = eye(3);
            p = v * t;
        else
            % 处理旋转关节
            w_norm = norm(w);
            w_unit = w / w_norm;
            w_unit_hat = skew(w_unit);
            
            % 计算旋转矩阵
            R = eye(3) + sin(w_norm * t) * w_unit_hat + ...
                (1 - cos(w_norm * t)) * w_unit_hat * w_unit_hat;
            
            % 计算平移向量
            p = (eye(3) - R) * (cross(w_unit, v / w_norm)) + ...
                (w_unit * w_unit' * v * t);
        end
        
        exp_m = [R, p; 0, 0, 0, 1];
    end

    % 内部函数：斜对称矩阵转换
    function S = skew(w)
        S = [0, -w(3), w(2); 
             w(3), 0, -w(1); 
             -w(2), w(1), 0];
    end
end


