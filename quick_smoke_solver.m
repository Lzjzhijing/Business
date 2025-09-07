function quick_smoke_solver()
    % 快速烟幕干扰弹投放策略求解器
    % 使用启发式算法快速求解
    
    clc; clear; close all;
    
    % 问题参数
    params = initialize_parameters();
    
    % 计算导弹轨迹
    missile_trajectories = calculate_missile_trajectories(params);
    
    % 使用启发式算法求解
    solution = heuristic_solve(params, missile_trajectories);
    
    % 生成结果
    generate_results(solution, params, missile_trajectories);
    
    % 可视化结果
    visualize_solution(solution, params, missile_trajectories);
    
    fprintf('快速求解完成！结果已保存到 result3.xlsx\n');
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

function solution = heuristic_solve(params, missile_trajectories)
    % 启发式算法求解
    
    % 计算导弹到达假目标的时间
    missile_arrival_times = zeros(params.num_missiles, 1);
    for i = 1:params.num_missiles
        distance = norm(params.missiles(i, :) - params.fake_target);
        missile_arrival_times(i) = distance / params.missile_speed;
    end
    
    fprintf('导弹到达时间: M1=%.1fs, M2=%.1fs, M3=%.1fs\n', ...
        missile_arrival_times(1), missile_arrival_times(2), missile_arrival_times(3));
    
    % 初始化解决方案
    solution = struct();
    solution.drone_headings = zeros(params.num_drones, 1);
    solution.drone_speeds = zeros(params.num_drones, 1);
    solution.smoke_times = zeros(params.num_drones, params.max_smoke_per_drone);
    
    % 为每架无人机分配任务
    for drone_idx = 1:params.num_drones
        % 计算无人机到各导弹轨迹的最优拦截点
        [best_heading, best_speed, smoke_schedule] = optimize_drone_mission(...
            drone_idx, params, missile_trajectories, missile_arrival_times);
        
        solution.drone_headings(drone_idx) = best_heading;
        solution.drone_speeds(drone_idx) = best_speed;
        solution.smoke_times(drone_idx, :) = smoke_schedule;
    end
    
    % 计算详细部署信息
    solution.deployment_details = calculate_deployment_details(solution, params);
end

function [best_heading, best_speed, smoke_schedule] = optimize_drone_mission(...
    drone_idx, params, missile_trajectories, missile_arrival_times)
    % 优化单架无人机的任务
    
    drone_pos = params.drones(drone_idx, :);
    best_coverage = 0;
    best_heading = 0;
    best_speed = params.drone_speed_min;
    best_schedule = zeros(1, params.max_smoke_per_drone);
    
    % 尝试不同的航向和速度组合
    heading_candidates = 0:30:330;  % 每30度一个候选
    speed_candidates = params.drone_speed_min:20:params.drone_speed_max;
    
    for heading = heading_candidates
        for speed = speed_candidates
            % 计算该配置下的最优烟幕弹投放时间
            [smoke_schedule, coverage] = optimize_smoke_timing(...
                drone_idx, heading, speed, params, missile_trajectories, missile_arrival_times);
            
            if coverage > best_coverage
                best_coverage = coverage;
                best_heading = heading;
                best_speed = speed;
                best_schedule = smoke_schedule;
            end
        end
    end
    
    smoke_schedule = best_schedule;
    fprintf('无人机FY%d: 航向=%.0f°, 速度=%.0fm/s, 覆盖率=%.2f%%\n', ...
        drone_idx, best_heading, best_speed, best_coverage*100);
end

function [smoke_schedule, coverage] = optimize_smoke_timing(...
    drone_idx, heading, speed, params, missile_trajectories, missile_arrival_times)
    % 优化烟幕弹投放时间
    
    smoke_schedule = zeros(1, params.max_smoke_per_drone);
    drone_pos = params.drones(drone_idx, :);
    direction = [cos(heading*pi/180), sin(heading*pi/180), 0];
    
    % 计算导弹轨迹与无人机航线的交点
    intersection_times = zeros(params.num_missiles, 1);
    intersection_positions = zeros(params.num_missiles, 3);
    
    for missile_idx = 1:params.num_missiles
        [t_intersect, pos_intersect] = find_intersection(...
            drone_pos, direction, speed, ...
            params.missiles(missile_idx, :), params.fake_target, params.missile_speed);
        
        if t_intersect > 0 && t_intersect <= params.max_simulation_time
            intersection_times(missile_idx) = t_intersect;
            intersection_positions(missile_idx, :) = pos_intersect;
        end
    end
    
    % 选择最佳的烟幕弹投放时间
    valid_intersections = find(intersection_times > 0);
    if isempty(valid_intersections)
        coverage = 0;
        return;
    end
    
    % 按时间排序
    [sorted_times, sort_idx] = sort(intersection_times(valid_intersections));
    
    % 分配烟幕弹
    smoke_count = 0;
    last_smoke_time = 0;
    
    for i = 1:min(length(sorted_times), params.max_smoke_per_drone)
        missile_idx = valid_intersections(sort_idx(i));
        smoke_time = sorted_times(i) - 2;  % 提前2秒投放
        
        if smoke_time > last_smoke_time + params.smoke_interval_min
            smoke_count = smoke_count + 1;
            smoke_schedule(smoke_count) = smoke_time;
            last_smoke_time = smoke_time;
        end
    end
    
    % 计算覆盖率
    coverage = calculate_coverage_for_drone(drone_idx, heading, speed, ...
        smoke_schedule, params, missile_trajectories);
end

function [t_intersect, pos_intersect] = find_intersection(...
    drone_pos, drone_direction, drone_speed, missile_pos, missile_target, missile_speed)
    % 计算无人机航线与导弹轨迹的交点
    
    missile_direction = missile_target - missile_pos;
    missile_direction = missile_direction / norm(missile_direction);
    
    % 求解交点时间
    % 无人机位置: drone_pos + drone_direction * drone_speed * t
    % 导弹位置: missile_pos + missile_direction * missile_speed * t
    
    % 建立方程组求解
    A = [drone_direction * drone_speed; -missile_direction * missile_speed]';
    b = missile_pos - drone_pos;
    
    if rank(A) == 2
        t_solution = A \ b;
        t_intersect = t_solution(1);
        
        if t_intersect > 0
            pos_intersect = drone_pos + drone_direction * drone_speed * t_intersect;
        else
            t_intersect = -1;
            pos_intersect = [0, 0, 0];
        end
    else
        t_intersect = -1;
        pos_intersect = [0, 0, 0];
    end
end

function coverage = calculate_coverage_for_drone(drone_idx, heading, speed, ...
    smoke_schedule, params, missile_trajectories)
    % 计算单架无人机的烟幕覆盖率
    
    coverage = 0;
    time_vec = missile_trajectories.time;
    drone_pos = params.drones(drone_idx, :);
    direction = [cos(heading*pi/180), sin(heading*pi/180), 0];
    
    for t_idx = 1:length(time_vec)
        t = time_vec(t_idx);
        
        % 检查是否有烟幕提供遮蔽
        smoke_coverage = 0;
        
        for smoke_idx = 1:params.max_smoke_per_drone
            smoke_time = smoke_schedule(smoke_idx);
            if smoke_time > 0 && t >= smoke_time
                % 计算烟幕弹投放位置
                drop_pos = drone_pos + direction * speed * smoke_time;
                
                % 计算烟幕云团位置（考虑下沉）
                elapsed_time = t - smoke_time;
                cloud_pos = drop_pos;
                cloud_pos(3) = cloud_pos(3) - params.smoke_sink_speed * elapsed_time;
                cloud_pos(3) = max(cloud_pos(3), 0);
                
                % 检查是否在有效时间内
                if elapsed_time <= params.smoke_effective_duration
                    % 检查是否覆盖任何导弹
                    for missile_idx = 1:params.num_missiles
                        missile_pos = squeeze(missile_trajectories.positions(t_idx, missile_idx, :))';
                        distance = norm(missile_pos - cloud_pos);
                        if distance <= params.smoke_effective_radius
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
    
    % 准备无人机数据
    drone_data = {};
    for i = 1:length(solution.deployment_details.drone_info)
        drone = solution.deployment_details.drone_info(i);
        drone_data{end+1, 1} = sprintf('FY%d', drone.drone_id);
        drone_data{end, 2} = drone.heading;
        drone_data{end, 3} = drone.speed;
        drone_data{end, 4} = drone.start_pos(1);
        drone_data{end, 5} = drone.start_pos(2);
        drone_data{end, 6} = drone.start_pos(3);
    end
    
    % 准备烟幕弹数据
    smoke_data = {};
    for i = 1:length(solution.deployment_details.smoke_info)
        smoke = solution.deployment_details.smoke_info(i);
        smoke_data{end+1, 1} = sprintf('FY%d', smoke.drone_id);
        smoke_data{end, 2} = smoke.drop_pos(1);
        smoke_data{end, 3} = smoke.drop_pos(2);
        smoke_data{end, 4} = smoke.drop_pos(3);
        smoke_data{end, 5} = smoke.burst_pos(1);
        smoke_data{end, 6} = smoke.burst_pos(2);
        smoke_data{end, 7} = smoke.burst_pos(3);
        smoke_data{end, 8} = smoke.effective_duration;
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
        coverage = calculate_coverage_for_drone(1, solution.drone_headings(1), ...
            solution.drone_speeds(1), solution.smoke_times(1, :), params, missile_trajectories);
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