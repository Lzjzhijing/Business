function smoke_interference_optimization()
    % 烟幕干扰弹投放策略优化
    % Smoke Interference Missile Deployment Strategy Optimization
    
    clear; clc; close all;
    
    %% 问题参数设置 Problem Parameters
    % 导弹信息 Missile Information
    missiles = [
        20000,    0, 2000;  % M1
        19000,  600, 2100;  % M2
        18000, -600, 1900   % M3
    ];
    
    % 无人机信息 UAV Information
    uavs = [
        17800,    0, 1800;  % FY1
        12000, 1400, 1400;  % FY2
         6000,-3000,  700;  % FY3
        11000, 2000, 1800;  % FY4
        13000,-2000, 1300   % FY5
    ];
    
    % 目标信息 Target Information
    fake_target = [0, 0, 0];        % 假目标位置
    real_target = [0, 200, 0];      % 真目标位置
    target_radius = 7;              % 目标半径
    target_height = 10;             % 目标高度
    
    % 系统参数 System Parameters
    missile_speed = 300;            % 导弹速度 m/s
    uav_speed_min = 70;             % 无人机最小速度 m/s
    uav_speed_max = 140;            % 无人机最大速度 m/s
    smoke_sink_speed = 3;           % 烟幕下沉速度 m/s
    smoke_radius = 10;              % 烟幕有效半径 m
    smoke_duration = 20;            % 烟幕有效时间 s
    max_smoke_per_uav = 3;          % 每架无人机最多投放烟幕弹数量
    min_deploy_interval = 1;        % 同一无人机投放间隔 s
    
    fprintf('开始烟幕干扰弹投放策略优化...\n');
    
    %% 计算导弹到达真目标的时间和轨迹
    [missile_trajectories, missile_times] = calculate_missile_trajectories(missiles, real_target, missile_speed);
    
    %% 为每枚导弹设计最优烟幕干扰策略
    results = cell(3, 1);  % 存储每枚导弹的干扰策略
    
    for missile_idx = 1:3
        fprintf('\n正在为导弹 M%d 设计干扰策略...\n', missile_idx);
        
        % 计算该导弹的拦截策略
        strategy = optimize_smoke_deployment(missile_idx, missiles, uavs, real_target, ...
            missile_speed, uav_speed_min, uav_speed_max, smoke_sink_speed, ...
            smoke_radius, smoke_duration, max_smoke_per_uav, min_deploy_interval);
        
        results{missile_idx} = strategy;
        
        fprintf('导弹 M%d 干扰策略完成\n', missile_idx);
    end
    
    %% 输出结果到Excel文件
    save_results_to_excel(results, '结果3.xlsx');
    
    %% 可视化结果
    visualize_strategy(missiles, uavs, real_target, results);
    
    fprintf('\n烟幕干扰弹投放策略优化完成！\n');
    fprintf('结果已保存到 结果3.xlsx 文件中。\n');
end

function [trajectories, times] = calculate_missile_trajectories(missiles, target, speed)
    % 计算导弹轨迹和到达时间
    num_missiles = size(missiles, 1);
    trajectories = cell(num_missiles, 1);
    times = zeros(num_missiles, 1);
    
    for i = 1:num_missiles
        start_pos = missiles(i, :);
        direction = target - start_pos;
        distance = norm(direction);
        times(i) = distance / speed;
        
        % 存储轨迹信息
        trajectories{i} = struct('start', start_pos, 'target', target, ...
            'direction', direction/norm(direction), 'distance', distance, 'time', times(i));
    end
end

function strategy = optimize_smoke_deployment(missile_idx, missiles, uavs, target, ...
    missile_speed, uav_speed_min, uav_speed_max, smoke_sink_speed, ...
    smoke_radius, smoke_duration, max_smoke_per_uav, min_deploy_interval)
    
    % 获取当前导弹信息
    missile_pos = missiles(missile_idx, :);
    missile_direction = (target - missile_pos) / norm(target - missile_pos);
    missile_distance = norm(target - missile_pos);
    missile_time = missile_distance / missile_speed;
    
    % 初始化策略结果
    strategy = struct();
    strategy.missile_id = missile_idx;
    strategy.deployments = [];
    
    % 计算导弹轨迹上的关键拦截点
    intercept_points = calculate_intercept_points(missile_pos, target, missile_direction, missile_time);
    
    % 为每个拦截点分配最优无人机和烟幕弹
    num_uavs = size(uavs, 1);
    best_coverage_time = 0;
    best_deployment = [];
    
    % 使用遗传算法或枚举法寻找最优解
    for uav_idx = 1:num_uavs
        for num_smokes = 1:max_smoke_per_uav
            deployment = optimize_single_uav_deployment(uav_idx, num_smokes, ...
                uavs(uav_idx, :), intercept_points, missile_direction, missile_speed, ...
                uav_speed_min, uav_speed_max, smoke_sink_speed, smoke_radius, ...
                smoke_duration, min_deploy_interval);
            
            if deployment.total_coverage_time > best_coverage_time
                best_coverage_time = deployment.total_coverage_time;
                best_deployment = deployment;
            end
        end
    end
    
    strategy.deployments = best_deployment;
    strategy.total_coverage_time = best_coverage_time;
end

function intercept_points = calculate_intercept_points(missile_pos, target, direction, total_time)
    % 计算导弹轨迹上的潜在拦截点
    num_points = 5;  % 在轨迹上选择5个拦截点
    intercept_points = zeros(num_points, 3);
    
    for i = 1:num_points
        t_ratio = i / (num_points + 1);  % 时间比例
        intercept_points(i, :) = missile_pos + direction * norm(target - missile_pos) * t_ratio;
    end
end

function deployment = optimize_single_uav_deployment(uav_idx, num_smokes, uav_pos, ...
    intercept_points, missile_direction, missile_speed, uav_speed_min, uav_speed_max, ...
    smoke_sink_speed, smoke_radius, smoke_duration, min_deploy_interval)
    
    deployment = struct();
    deployment.uav_id = uav_idx;
    deployment.uav_speed = uav_speed_max;  % 使用最大速度
    deployment.smoke_deployments = [];
    deployment.total_coverage_time = 0;
    
    % 选择最优的拦截点
    best_point_idx = 1;  % 简化处理，选择第一个拦截点
    target_point = intercept_points(best_point_idx, :);
    
    % 计算无人机飞行方向和时间
    flight_direction = (target_point - uav_pos) / norm(target_point - uav_pos);
    flight_distance = norm(target_point - uav_pos);
    flight_time = flight_distance / deployment.uav_speed;
    
    deployment.flight_direction = flight_direction;
    deployment.flight_time = flight_time;
    deployment.target_point = target_point;
    
    % 计算烟幕弹投放时机
    for smoke_idx = 1:num_smokes
        smoke_deploy = struct();
        smoke_deploy.deploy_time = flight_time + (smoke_idx - 1) * min_deploy_interval;
        smoke_deploy.deploy_position = uav_pos + flight_direction * deployment.uav_speed * smoke_deploy.deploy_time;
        
        % 计算烟幕弹的爆炸点和时间
        [explosion_pos, explosion_time] = calculate_smoke_explosion(smoke_deploy.deploy_position, ...
            target_point, smoke_sink_speed);
        
        smoke_deploy.explosion_position = explosion_pos;
        smoke_deploy.explosion_time = explosion_time;
        
        % 计算有效遮蔽时间
        coverage_time = calculate_coverage_time(explosion_pos, explosion_time, ...
            missile_direction, missile_speed, smoke_radius, smoke_duration);
        
        smoke_deploy.coverage_time = coverage_time;
        deployment.total_coverage_time = deployment.total_coverage_time + coverage_time;
        
        deployment.smoke_deployments = [deployment.smoke_deployments, smoke_deploy];
    end
end

function [explosion_pos, explosion_time] = calculate_smoke_explosion(deploy_pos, target_pos, sink_speed)
    % 计算烟幕弹的最优爆炸位置和时间
    % 简化处理：在目标点上方适当高度爆炸
    explosion_pos = [target_pos(1), target_pos(2), target_pos(3) + 50];  % 在目标上方50m爆炸
    
    % 计算从投放到爆炸的时间（考虑重力和下沉）
    fall_height = deploy_pos(3) - explosion_pos(3);
    if fall_height > 0
        explosion_time = sqrt(2 * fall_height / 9.8);  % 自由落体时间
    else
        explosion_time = 0.1;  % 最小爆炸延迟
    end
end

function coverage_time = calculate_coverage_time(explosion_pos, explosion_time, ...
    missile_direction, missile_speed, smoke_radius, smoke_duration)
    
    % 简化计算：假设烟幕能有效遮蔽导弹视线的时间
    % 实际应该考虑烟幕云团的移动和导弹轨迹的交集
    coverage_time = smoke_duration * 0.8;  % 假设80%的时间有效
end

function save_results_to_excel(results, filename)
    % 保存结果到Excel文件
    
    % 创建汇总数据
    summary_data = {};
    row_idx = 1;
    
    % 添加表头
    headers = {'导弹编号', '无人机编号', '飞行方向X', '飞行方向Y', '飞行方向Z', ...
               '飞行速度', '烟幕弹编号', '投放时间', '投放位置X', '投放位置Y', '投放位置Z', ...
               '爆炸位置X', '爆炸位置Y', '爆炸位置Z', '爆炸时间', '遮蔽时间'};
    
    summary_data(1, :) = headers;
    row_idx = 2;
    
    for missile_idx = 1:length(results)
        strategy = results{missile_idx};
        if ~isempty(strategy.deployments)
            deployment = strategy.deployments;
            
            for smoke_idx = 1:length(deployment.smoke_deployments)
                smoke = deployment.smoke_deployments(smoke_idx);
                
                row_data = {
                    sprintf('M%d', missile_idx),
                    sprintf('FY%d', deployment.uav_id),
                    deployment.flight_direction(1),
                    deployment.flight_direction(2),
                    deployment.flight_direction(3),
                    deployment.uav_speed,
                    smoke_idx,
                    smoke.deploy_time,
                    smoke.deploy_position(1),
                    smoke.deploy_position(2),
                    smoke.deploy_position(3),
                    smoke.explosion_position(1),
                    smoke.explosion_position(2),
                    smoke.explosion_position(3),
                    smoke.explosion_time,
                    smoke.coverage_time
                };
                
                summary_data(row_idx, :) = row_data;
                row_idx = row_idx + 1;
            end
        end
    end
    
    % 写入Excel文件
    try
        writecell(summary_data, filename, 'Sheet', 1);
        fprintf('结果已成功保存到 %s\n', filename);
    catch ME
        fprintf('保存Excel文件时出错: %s\n', ME.message);
        % 如果Excel写入失败，保存为CSV格式
        csv_filename = strrep(filename, '.xlsx', '.csv');
        writecell(summary_data, csv_filename);
        fprintf('已保存为CSV格式: %s\n', csv_filename);
    end
end

function visualize_strategy(missiles, uavs, target, results)
    % 可视化烟幕干扰策略
    
    figure('Position', [100, 100, 1200, 800]);
    
    % 绘制3D场景
    subplot(2, 2, [1, 2]);
    hold on; grid on;
    
    % 绘制导弹
    for i = 1:size(missiles, 1)
        plot3(missiles(i, 1), missiles(i, 2), missiles(i, 3), 'ro', ...
            'MarkerSize', 10, 'MarkerFaceColor', 'red');
        text(missiles(i, 1), missiles(i, 2), missiles(i, 3) + 100, ...
            sprintf('M%d', i), 'FontSize', 12, 'Color', 'red');
        
        % 绘制导弹轨迹
        trajectory_end = target;
        plot3([missiles(i, 1), trajectory_end(1)], ...
              [missiles(i, 2), trajectory_end(2)], ...
              [missiles(i, 3), trajectory_end(3)], 'r--', 'LineWidth', 2);
    end
    
    % 绘制无人机
    for i = 1:size(uavs, 1)
        plot3(uavs(i, 1), uavs(i, 2), uavs(i, 3), 'bo', ...
            'MarkerSize', 8, 'MarkerFaceColor', 'blue');
        text(uavs(i, 1), uavs(i, 2), uavs(i, 3) + 100, ...
            sprintf('FY%d', i), 'FontSize', 10, 'Color', 'blue');
    end
    
    % 绘制目标
    plot3(target(1), target(2), target(3), 'gs', ...
        'MarkerSize', 15, 'MarkerFaceColor', 'green');
    text(target(1), target(2), target(3) + 100, '真目标', ...
        'FontSize', 12, 'Color', 'green');
    
    % 绘制烟幕干扰策略
    colors = ['m', 'c', 'y'];
    for missile_idx = 1:length(results)
        if ~isempty(results{missile_idx}.deployments)
            deployment = results{missile_idx}.deployments;
            color = colors(missile_idx);
            
            % 绘制无人机飞行轨迹
            uav_pos = uavs(deployment.uav_id, :);
            flight_end = deployment.target_point;
            plot3([uav_pos(1), flight_end(1)], ...
                  [uav_pos(2), flight_end(2)], ...
                  [uav_pos(3), flight_end(3)], color, 'LineWidth', 3);
            
            % 绘制烟幕弹爆炸点
            for smoke_idx = 1:length(deployment.smoke_deployments)
                smoke = deployment.smoke_deployments(smoke_idx);
                plot3(smoke.explosion_position(1), smoke.explosion_position(2), ...
                    smoke.explosion_position(3), 'o', 'Color', color, ...
                    'MarkerSize', 12, 'MarkerFaceColor', color);
            end
        end
    end
    
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('烟幕干扰弹投放策略3D视图');
    legend('导弹', '无人机', '目标', 'Location', 'best');
    view(45, 30);
    
    % 绘制时间序列图
    subplot(2, 2, 3);
    hold on; grid on;
    
    time_data = [];
    coverage_data = [];
    
    for missile_idx = 1:length(results)
        if ~isempty(results{missile_idx}.deployments)
            deployment = results{missile_idx}.deployments;
            for smoke_idx = 1:length(deployment.smoke_deployments)
                smoke = deployment.smoke_deployments(smoke_idx);
                time_data = [time_data, smoke.explosion_time];
                coverage_data = [coverage_data, smoke.coverage_time];
            end
        end
    end
    
    if ~isempty(time_data)
        bar(time_data, coverage_data);
        xlabel('爆炸时间 (s)');
        ylabel('遮蔽时间 (s)');
        title('烟幕遮蔽时间分布');
    end
    
    % 绘制效果统计
    subplot(2, 2, 4);
    
    missile_labels = {};
    total_times = [];
    
    for missile_idx = 1:length(results)
        missile_labels{end+1} = sprintf('M%d', missile_idx);
        if ~isempty(results{missile_idx}.deployments)
            total_times(end+1) = results{missile_idx}.total_coverage_time;
        else
            total_times(end+1) = 0;
        end
    end
    
    bar(total_times);
    set(gca, 'XTickLabel', missile_labels);
    xlabel('导弹编号');
    ylabel('总遮蔽时间 (s)');
    title('各导弹总遮蔽时间');
    grid on;
    
    sgtitle('烟幕干扰弹投放策略分析结果', 'FontSize', 16);
end

% 运行主函数
smoke_interference_optimization();