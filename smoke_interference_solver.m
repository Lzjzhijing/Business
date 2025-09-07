function smoke_interference_solver()
    % 无人机烟幕干扰弹投放策略优化求解器
    % 解决多导弹来袭情况下的最优烟幕弹投放策略
    
    clc; clear; close all;
    
    % 问题参数
    params = initialize_parameters();
    
    % 计算导弹轨迹
    missile_trajectories = calculate_missile_trajectories(params);
    
    % 优化求解
    [optimal_solution, performance] = optimize_deployment_strategy(params, missile_trajectories);
    
    % 生成结果
    generate_results(optimal_solution, params, missile_trajectories);
    
    % 可视化结果
    visualize_solution(optimal_solution, params, missile_trajectories);
    
    fprintf('优化完成！结果已保存到 result3.xlsx\n');
end

function params = initialize_parameters()
    % 初始化问题参数
    
    % 导弹初始位置 (x, y, z)
    params.missiles = [
        20000, 0, 2000;      % M1
        19000, 600, 2100;    % M2
        18000, -600, 1900    % M3
    ];
    
    % 无人机初始位置 (x, y, z)
    params.drones = [
        17800, 0, 1800;      % FY1
        12000, 1400, 1400;   % FY2
        6000, -3000, 700;    % FY3
        11000, 2000, 1800;   % FY4
        13000, -2000, 1300   % FY5
    ];
    
    % 目标参数
    params.target_center = [0, 200, 0];  % 真目标中心
    params.target_radius = 7;            % 真目标半径
    params.target_height = 10;           % 真目标高度
    params.fake_target = [0, 0, 0];      % 假目标位置
    
    % 物理参数
    params.missile_speed = 300;          % 导弹速度 m/s
    params.drone_speed_min = 70;         % 无人机最小速度 m/s
    params.drone_speed_max = 140;        % 无人机最大速度 m/s
    params.smoke_sink_speed = 3;         % 烟幕下沉速度 m/s
    params.smoke_effective_radius = 10;  % 烟幕有效半径 m
    params.smoke_effective_duration = 20; % 烟幕有效持续时间 s
    params.smoke_interval_min = 1;       % 烟幕弹投放最小间隔 s
    
    % 约束参数
    params.max_smoke_per_drone = 3;      % 每架无人机最多投放烟幕弹数量
    params.num_drones = size(params.drones, 1);
    params.num_missiles = size(params.missiles, 1);
    
    % 时间参数
    params.time_step = 0.1;              % 时间步长 s
    params.max_simulation_time = 100;    % 最大仿真时间 s
end

function trajectories = calculate_missile_trajectories(params)
    % 计算导弹轨迹
    
    trajectories = struct();
    trajectories.time = 0:params.time_step:params.max_simulation_time;
    trajectories.positions = zeros(length(trajectories.time), params.num_missiles, 3);
    
    for i = 1:params.num_missiles
        start_pos = params.missiles(i, :);
        % 导弹直指假目标
        direction = params.fake_target - start_pos;
        direction = direction / norm(direction);
        
        for t_idx = 1:length(trajectories.time)
            t = trajectories.time(t_idx);
            pos = start_pos + direction * params.missile_speed * t;
            trajectories.positions(t_idx, i, :) = pos;
        end
    end
end

function [solution, performance] = optimize_deployment_strategy(params, missile_trajectories)
    % 优化烟幕弹投放策略
    
    % 使用遗传算法优化
    options = optimoptions('ga', ...
        'Display', 'iter', ...
        'MaxGenerations', 100, ...
        'PopulationSize', 200, ...
        'UseParallel', true);
    
    % 决策变量：[无人机航向(5个), 无人机速度(5个), 烟幕弹投放时间(15个)]
    % 每架无人机最多3枚烟幕弹，共15个投放时间
    nvars = 5 + 5 + 15;  % 航向 + 速度 + 投放时间
    
    % 变量边界
    lb = [zeros(1,5), params.drone_speed_min*ones(1,5), zeros(1,15)];
    ub = [360*ones(1,5), params.drone_speed_max*ones(1,5), params.max_simulation_time*ones(1,15)];
    
    % 整数约束（航向为整数度）
    intcon = 1:5;
    
    % 优化目标函数
    fitness_func = @(x) objective_function(x, params, missile_trajectories);
    
    % 约束函数
    constraint_func = @(x) constraint_function(x, params);
    
    % 运行优化
    [x_opt, fval, exitflag] = ga(fitness_func, nvars, [], [], [], [], lb, ub, constraint_func, intcon, options);
    
    % 解析最优解
    solution = parse_solution(x_opt, params);
    performance = fval;
    
    fprintf('优化完成，目标函数值: %.4f\n', fval);
end

function fval = objective_function(x, params, missile_trajectories)
    % 目标函数：最大化烟幕遮蔽效果
    
    % 解析决策变量
    drone_headings = x(1:5);           % 无人机航向
    drone_speeds = x(6:10);            % 无人机速度
    smoke_times = x(11:25);            % 烟幕弹投放时间
    
    % 计算烟幕遮蔽效果
    total_coverage = 0;
    
    for missile_idx = 1:params.num_missiles
        missile_coverage = calculate_missile_coverage(missile_idx, drone_headings, ...
            drone_speeds, smoke_times, params, missile_trajectories);
        total_coverage = total_coverage + missile_coverage;
    end
    
    % 目标函数为负值（因为要最大化）
    fval = -total_coverage;
end

function coverage = calculate_missile_coverage(missile_idx, drone_headings, ...
    drone_speeds, smoke_times, params, missile_trajectories)
    % 计算对特定导弹的烟幕遮蔽效果
    
    coverage = 0;
    time_vec = missile_trajectories.time;
    
    for t_idx = 1:length(time_vec)
        t = time_vec(t_idx);
        missile_pos = squeeze(missile_trajectories.positions(t_idx, missile_idx, :))';
        
        % 检查是否有烟幕提供遮蔽
        smoke_coverage = 0;
        
        for drone_idx = 1:params.num_drones
            for smoke_idx = 1:params.max_smoke_per_drone
                smoke_time_idx = (drone_idx-1) * params.max_smoke_per_drone + smoke_idx;
                smoke_time = smoke_times(smoke_time_idx);
                
                if smoke_time > 0 && t >= smoke_time
                    % 计算烟幕弹投放位置
                    smoke_drop_pos = calculate_smoke_drop_position(drone_idx, drone_headings, ...
                        drone_speeds, smoke_time, params);
                    
                    % 计算烟幕云团位置（考虑下沉）
                    smoke_cloud_pos = calculate_smoke_cloud_position(smoke_drop_pos, ...
                        t - smoke_time, params);
                    
                    % 检查是否在有效遮蔽范围内
                    distance = norm(missile_pos - smoke_cloud_pos);
                    if distance <= params.smoke_effective_radius
                        % 检查是否在有效时间内
                        if (t - smoke_time) <= params.smoke_effective_duration
                            smoke_coverage = 1;
                            break;
                        end
                    end
                end
            end
            if smoke_coverage > 0
                break;
            end
        end
        
        coverage = coverage + smoke_coverage;
    end
    
    % 归一化覆盖率
    coverage = coverage / length(time_vec);
end

function drop_pos = calculate_smoke_drop_position(drone_idx, drone_headings, ...
    drone_speeds, smoke_time, params)
    % 计算烟幕弹投放位置
    
    start_pos = params.drones(drone_idx, :);
    heading = drone_headings(drone_idx) * pi / 180;  % 转换为弧度
    speed = drone_speeds(drone_idx);
    
    % 无人机飞行方向
    direction = [cos(heading), sin(heading), 0];
    
    % 投放位置
    drop_pos = start_pos + direction * speed * smoke_time;
end

function cloud_pos = calculate_smoke_cloud_position(drop_pos, elapsed_time, params)
    % 计算烟幕云团位置（考虑重力下沉）
    
    cloud_pos = drop_pos;
    cloud_pos(3) = cloud_pos(3) - params.smoke_sink_speed * elapsed_time;
    
    % 确保不会沉到地面以下
    cloud_pos(3) = max(cloud_pos(3), 0);
end

function [c, ceq] = constraint_function(x, params)
    % 约束函数
    
    c = [];
    ceq = [];
    
    % 解析决策变量
    drone_headings = x(1:5);
    drone_speeds = x(6:10);
    smoke_times = x(11:25);
    
    % 约束1：每架无人机的烟幕弹投放时间必须递增
    for drone_idx = 1:params.num_drones
        drone_smoke_times = smoke_times((drone_idx-1)*3+1:drone_idx*3);
        valid_times = drone_smoke_times(drone_smoke_times > 0);
        if length(valid_times) > 1
            for i = 2:length(valid_times)
                c = [c, valid_times(i-1) - valid_times(i) + params.smoke_interval_min];
            end
        end
    end
    
    % 约束2：烟幕弹投放时间不能超过仿真时间
    c = [c, smoke_times - params.max_simulation_time];
end

function solution = parse_solution(x_opt, params)
    % 解析最优解
    
    solution = struct();
    solution.drone_headings = x_opt(1:5);
    solution.drone_speeds = x_opt(6:10);
    solution.smoke_times = reshape(x_opt(11:25), 5, 3);
    
    % 计算详细的投放信息
    solution.deployment_details = calculate_deployment_details(solution, params);
end

function details = calculate_deployment_details(solution, params)
    % 计算详细的投放信息
    
    details = struct();
    details.drone_info = [];
    details.smoke_info = [];
    
    for drone_idx = 1:params.num_drones
        start_pos = params.drones(drone_idx, :);
        heading = solution.drone_headings(drone_idx);
        speed = solution.drone_speeds(drone_idx);
        
        % 无人机信息
        drone_info = struct();
        drone_info.drone_id = drone_idx;
        drone_info.heading = heading;
        drone_info.speed = speed;
        drone_info.start_pos = start_pos;
        
        details.drone_info = [details.drone_info; drone_info];
        
        % 烟幕弹信息
        for smoke_idx = 1:params.max_smoke_per_drone
            smoke_time = solution.smoke_times(drone_idx, smoke_idx);
            if smoke_time > 0
                % 计算投放位置
                direction = [cos(heading*pi/180), sin(heading*pi/180), 0];
                drop_pos = start_pos + direction * speed * smoke_time;
                
                % 计算起爆位置（假设瞬时起爆）
                burst_pos = drop_pos;
                
                smoke_info = struct();
                smoke_info.drone_id = drone_idx;
                smoke_info.smoke_id = smoke_idx;
                smoke_info.drop_time = smoke_time;
                smoke_info.drop_pos = drop_pos;
                smoke_info.burst_pos = burst_pos;
                smoke_info.effective_duration = params.smoke_effective_duration;
                
                details.smoke_info = [details.smoke_info; smoke_info];
            end
        end
    end
end

function generate_results(solution, params, missile_trajectories)
    % 生成Excel结果文件
    
    % 准备数据
    drone_data = [];
    smoke_data = [];
    
    % 无人机数据
    for i = 1:length(solution.deployment_details.drone_info)
        drone = solution.deployment_details.drone_info(i);
        drone_data = [drone_data; {sprintf('FY%d', drone.drone_id), ...
            drone.heading, drone.speed, drone.start_pos(1), drone.start_pos(2), drone.start_pos(3)}];
    end
    
    % 烟幕弹数据
    for i = 1:length(solution.deployment_details.smoke_info)
        smoke = solution.deployment_details.smoke_info(i);
        drone_id = sprintf('FY%d', smoke.drone_id);
        smoke_data = [smoke_data; {drone_id, smoke.drop_pos(1), smoke.drop_pos(2), smoke.drop_pos(3), ...
            smoke.burst_pos(1), smoke.burst_pos(2), smoke.burst_pos(3), smoke.effective_duration}];
    end
    
    % 创建Excel文件
    filename = 'result3.xlsx';
    
    % 无人机工作表
    drone_headers = {'无人机编号', '航向(度)', '飞行速度(m/s)', '起始X(m)', '起始Y(m)', '起始Z(m)'};
    xlswrite(filename, [drone_headers; drone_data], '无人机信息');
    
    % 烟幕弹工作表
    smoke_headers = {'无人机编号', '投放点X(m)', '投放点Y(m)', '投放点Z(m)', ...
        '起爆点X(m)', '起爆点Y(m)', '起爆点Z(m)', '有效遮蔽时长(s)'};
    xlswrite(filename, [smoke_headers; smoke_data], '烟幕弹信息');
    
    fprintf('结果已保存到 %s\n', filename);
end

function visualize_solution(solution, params, missile_trajectories)
    % 可视化解决方案
    
    figure('Position', [100, 100, 1200, 800]);
    
    % 3D视图
    subplot(2,2,1);
    hold on;
    
    % 绘制导弹轨迹
    colors = ['r', 'g', 'b'];
    for i = 1:params.num_missiles
        trajectory = squeeze(missile_trajectories.positions(:, i, :));
        plot3(trajectory(:,1), trajectory(:,2), trajectory(:,3), colors(i), 'LineWidth', 2);
        plot3(params.missiles(i,1), params.missiles(i,2), params.missiles(i,3), ...
            [colors(i) 'o'], 'MarkerSize', 8, 'MarkerFaceColor', colors(i));
    end
    
    % 绘制目标
    plot3(params.target_center(1), params.target_center(2), params.target_center(3), ...
        'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    plot3(params.fake_target(1), params.fake_target(2), params.fake_target(3), ...
        'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'y');
    
    % 绘制无人机轨迹和烟幕弹投放
    for i = 1:length(solution.deployment_details.drone_info)
        drone = solution.deployment_details.drone_info(i);
        start_pos = drone.start_pos;
        direction = [cos(drone.heading*pi/180), sin(drone.heading*pi/180), 0];
        
        % 无人机轨迹
        t_end = max(solution.smoke_times(i, :));
        if t_end > 0
            t_vec = 0:0.1:t_end;
            drone_trajectory = start_pos + direction * drone.speed * t_vec';
            plot3(drone_trajectory(:,1), drone_trajectory(:,2), drone_trajectory(:,3), ...
                'k--', 'LineWidth', 1);
        end
        
        % 烟幕弹投放点
        for j = 1:params.max_smoke_per_drone
            smoke_time = solution.smoke_times(i, j);
            if smoke_time > 0
                drop_pos = start_pos + direction * drone.speed * smoke_time;
                plot3(drop_pos(1), drop_pos(2), drop_pos(3), 'ro', 'MarkerSize', 6);
            end
        end
    end
    
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('3D 轨迹视图');
    legend('M1轨迹', 'M2轨迹', 'M3轨迹', '真目标', '假目标', '无人机轨迹', '烟幕弹投放点');
    grid on; axis equal;
    
    % 俯视图
    subplot(2,2,2);
    hold on;
    
    % 绘制导弹轨迹（俯视）
    for i = 1:params.num_missiles
        trajectory = squeeze(missile_trajectories.positions(:, i, :));
        plot(trajectory(:,1), trajectory(:,2), colors(i), 'LineWidth', 2);
        plot(params.missiles(i,1), params.missiles(i,2), [colors(i) 'o'], 'MarkerSize', 8);
    end
    
    % 绘制目标（俯视）
    plot(params.target_center(1), params.target_center(2), 'ko', 'MarkerSize', 10);
    plot(params.fake_target(1), params.fake_target(2), 'ks', 'MarkerSize', 10);
    
    % 绘制无人机和烟幕弹投放（俯视）
    for i = 1:length(solution.deployment_details.drone_info)
        drone = solution.deployment_details.drone_info(i);
        plot(params.drones(i,1), params.drones(i,2), 'b^', 'MarkerSize', 8);
        
        for j = 1:params.max_smoke_per_drone
            smoke_time = solution.smoke_times(i, j);
            if smoke_time > 0
                direction = [cos(drone.heading*pi/180), sin(drone.heading*pi/180), 0];
                drop_pos = params.drones(i, :) + direction * drone.speed * smoke_time;
                plot(drop_pos(1), drop_pos(2), 'ro', 'MarkerSize', 6);
            end
        end
    end
    
    xlabel('X (m)'); ylabel('Y (m)');
    title('俯视图');
    grid on; axis equal;
    
    % 时间轴视图
    subplot(2,2,3);
    hold on;
    
    % 绘制导弹到达时间
    for i = 1:params.num_missiles
        arrival_time = norm(params.missiles(i, :) - params.fake_target) / params.missile_speed;
        plot([0, arrival_time], [i, i], colors(i), 'LineWidth', 3);
        text(arrival_time, i, sprintf('M%d', i), 'FontSize', 10);
    end
    
    % 绘制烟幕弹投放时间
    for i = 1:params.num_drones
        for j = 1:params.max_smoke_per_drone
            smoke_time = solution.smoke_times(i, j);
            if smoke_time > 0
                plot(smoke_time, i+3, 'ro', 'MarkerSize', 6);
                text(smoke_time, i+3, sprintf('FY%d', i), 'FontSize', 8);
            end
        end
    end
    
    xlabel('时间 (s)'); ylabel('导弹/无人机');
    title('时间轴视图');
    grid on;
    
    % 性能统计
    subplot(2,2,4);
    
    % 计算覆盖率统计
    total_coverage = 0;
    for missile_idx = 1:params.num_missiles
        coverage = calculate_missile_coverage(missile_idx, solution.drone_headings, ...
            solution.drone_speeds, solution.smoke_times(:), params, missile_trajectories);
        total_coverage = total_coverage + coverage;
    end
    
    avg_coverage = total_coverage / params.num_missiles;
    
    % 显示统计信息
    text(0.1, 0.8, sprintf('平均覆盖率: %.2f%%', avg_coverage*100), 'FontSize', 12);
    text(0.1, 0.6, sprintf('总烟幕弹数: %d', sum(solution.smoke_times(:) > 0)), 'FontSize', 12);
    text(0.1, 0.4, sprintf('平均投放时间: %.1fs', mean(solution.smoke_times(solution.smoke_times > 0))), 'FontSize', 12);
    
    axis off;
    title('性能统计');
    
    sgtitle('无人机烟幕干扰弹投放策略优化结果');
end