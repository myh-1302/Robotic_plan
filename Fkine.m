% AUTHOR :
%
% ABSTRACT： 这是计算机器人正解函数，通用函数，需调用Transformation函数
% 
% INPUT： Xi      机器人各关节运动旋量
%         theta   机器人关节位移，1xN向量，单位m和rad
%         g0      机器人基准参考位姿， 4X4矩阵
% OUTPUT: g_st    机器人末端位姿位姿， 4X4矩阵
% 
function g_st = Fkine(Xi, theta, g0)
    n = length(theta);
    g_st = eye(4);
    
    for i = 1:n
        % 计算当前关节的指数映射
        exp_m = exp_w(Xi(:, i), theta(i));
        % 连乘得到总变换矩阵
        g_st = g_st * exp_m;
    end
    
    % 乘以初始位姿
    g_st = g_st * g0;
end

function exp_m = exp_w(x, t)
    v = x(1:3);  % 线速度部分
    w = x(4:6);  % 角速度部分

    if norm(w) < eps
        % 对于移动关节
        R = eye(3);
        p = v * t;
    else
        % 对于旋转关节
        w_norm = norm(w);
        w_unit = w / w_norm;
        w_unit_hat = skew(w_unit);
        
        % 正确的旋转矩阵计算
        R = eye(3) + sin(w_norm * t) * w_unit_hat + ...
            (1 - cos(w_norm * t)) * w_unit_hat * w_unit_hat;
        
        % 正确的平移向量计算
        p = (eye(3) - R) * (cross(w_unit, v / w_norm)) + ...
            (w_unit * w_unit' * v * t);
    end
    
    exp_m = [R, p; 0, 0, 0, 1];
end

function S = skew(w)
    S = [0, -w(3), w(2); 
         w(3), 0, -w(1); 
         -w(2), w(1), 0];
end