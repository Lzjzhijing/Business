function advanced_smoke_optimization()
    % 高级烟幕干扰弹投放策略优化
    % Advanced Smoke Interference Missile Deployment Strategy Optimization
    
    clear; clc; close all;
    
    %% 问题参数设置
    params = setup_parameters();
    
    fprintf('=== 烟幕干扰弹投放策略优化系统 ===\n');
    fprintf('导弹数量: %d\n', size(params.missiles, 1));
    fprintf('无人机数量: %d\n', size(params.uavs, 1));
    fprintf('每架无人机最多投放: %d 枚烟幕弹\n', params.max_smoke_per_uav);
    
    %% 主优化算法
    [optimal_strategy, performance_metrics] = solve_optimization_problem(params);
    
    %% 结果输出
    save_results_to_excel(optimal_strategy, performance_metrics, '结果3.xlsx');
    
    %% 可视化和分析
    create_comprehensive_visualization(params, optimal_strategy, performance_metrics);
    
    %% 输出策略摘要
    print_strategy_summary(optimal_strategy, performance_metrics);
    
    fprintf('\n优化完成！结果已保存到 结果3.xlsx\n');
end

function params = setup_parameters()
    % 设置所有问题参数
    params = struct();
    
    % 导弹信息 (位置坐标)
    params.missiles = [
        20000,    0, 2000;  % M1
        19000,  600, 2100;  % M2
        18000, -600, 1900   % M3
    ];
    
    % 无人机信息 (位置坐标)
    params.uavs = [
        17800,    0, 1800;  % FY1
        12000, 1400, 1400;  % FY2
         6000,-3000,  700;  % FY3
        11000, 2000, 1800;  % FY4
        13000,-2000, 1300   % FY5
    ];
    
    % 目标信息
    params.fake_target = [0, 0, 0];
    params.real_target = [0, 200, 0];
    params.target_radius = 7;
    params.target_height = 10;
    
    % 物理参数
    params.missile_speed = 300;         % m/s
    params.uav_speed_min = 70;          % m/s
    params.uav_speed_max = 140;         % m/s
    params.smoke_sink_speed = 3;        % m/s
    params.smoke_radius = 10;           % m
    params.smoke_duration = 20;         % s
    params.gravity = 9.8;               % m/s²
    
    % 约束条件
    params.max_smoke_per_uav = 3;
    params.min_deploy_interval = 1;     % s
    
    % 优化参数
    params.num_intercept_points = 10;   % 每条导弹轨迹的拦截点数量
    params.optimization_iterations = 100;
end

function [optimal_strategy, metrics] = solve_optimization_problem(params)
    % 主优化求解函数
    
    fprintf('\n开始优化求解...\n');
    
    % 计算所有导弹的轨迹信息
    missile_trajectories = calculate_all_missile_trajectories(params);
    
    % 初始化最优策略
    optimal_strategy = initialize_strategy(params);
    best_objective = -inf;
    
    % 使用混合优化算法
    for iter = 1:params.optimization_iterations
        if mod(iter, 20) == 0
            fprintf('优化进度: %d/%d\n', iter, params.optimization_iterations);
        end
        
        % 生成候选策略
        candidate_strategy = generate_candidate_strategy(params, missile_trajectories, iter);
        
        % 评估策略性能
        objective_value = evaluate_strategy(candidate_strategy, params, missile_trajectories);
        
        % 更新最优策略
        if objective_value > best_objective
            best_objective = objective_value;
            optimal_strategy = candidate_strategy;
        end
    end
    
    % 计算性能指标
    metrics = calculate_performance_metrics(optimal_strategy, params, missile_trajectories);
    metrics.best_objective = best_objective;
    
    fprintf('优化完成！最优目标值: %.2f\n', best_objective);
end

function trajectories = calculate_all_missile_trajectories(params)
    % 计算所有导弹的详细轨迹信息
    
    num_missiles = size(params.missiles, 1);
    trajectories = cell(num_missiles, 1);
    
    for i = 1:num_missiles
        traj = struct();
        traj.start_pos = params.missiles(i, :);
        traj.target_pos = params.real_target;
        traj.direction = (traj.target_pos - traj.start_pos);
        traj.distance = norm(traj.direction);
        traj.direction = traj.direction / traj.distance;
        traj.flight_time = traj.distance / params.missile_speed;
        
        % 计算轨迹上的关键点
        traj.intercept_points = zeros(params.num_intercept_points, 3);
        for j = 1:params.num_intercept_points
            t_ratio = j / (params.num_intercept_points + 1);
            traj.intercept_points(j, :) = traj.start_pos + traj.direction * traj.distance * t_ratio;
        end
        
        trajectories{i} = traj;
    end
end

function strategy = initialize_strategy(params)
    % 初始化策略结构
    strategy = struct();
    strategy.assignments = [];  % UAV分配矩阵
    strategy.deployments = [];  % 具体部署方案
    
    num_missiles = size(params.missiles, 1);
    num_uavs = size(params.uavs, 1);
    
    for m = 1:num_missiles
        missile_strategy = struct();
        missile_strategy.missile_id = m;
        missile_strategy.assigned_uavs = [];
        strategy.deployments = [strategy.deployments, missile_strategy];
    end
end

function candidate = generate_candidate_strategy(params, trajectories, iteration)
    % 生成候选策略
    
    candidate = initialize_strategy(params);
    num_missiles = length(trajectories);
    num_uavs = size(params.uavs, 1);
    
    % 为每枚导弹分配无人机和制定部署方案
    for m = 1:num_missiles
        % 随机选择1-2架无人机参与拦截
        num_assigned_uavs = randi([1, min(2, num_uavs)]);
        available_uavs = randperm(num_uavs, num_assigned_uavs);
        
        missile_deployment = struct();
        missile_deployment.missile_id = m;
        missile_deployment.uav_deployments = [];
        
        for u_idx = 1:num_assigned_uavs
            uav_id = available_uavs(u_idx);
            uav_deployment = design_uav_deployment(uav_id, m, params, trajectories{m}, iteration);
            missile_deployment.uav_deployments = [missile_deployment.uav_deployments, uav_deployment];
        end
        
        candidate.deployments(m) = missile_deployment;
    end
end

function uav_deployment = design_uav_deployment(uav_id, missile_id, params, trajectory, iteration)
    % 为特定无人机设计部署方案
    
    uav_deployment = struct();
    uav_deployment.uav_id = uav_id;
    uav_deployment.missile_id = missile_id;
    
    % 无人机当前位置
    uav_pos = params.uavs(uav_id, :);
    
    % 选择最优拦截点 (基于距离和时间的权衡)
    best_intercept_idx = select_optimal_intercept_point(uav_pos, trajectory, params, iteration);
    target_point = trajectory.intercept_points(best_intercept_idx, :);
    
    % 计算飞行参数
    flight_vector = target_point - uav_pos;
    flight_distance = norm(flight_vector);
    flight_direction = flight_vector / flight_distance;
    
    % 优化飞行速度 (在约束范围内)
    optimal_speed = optimize_uav_speed(flight_distance, trajectory.flight_time, params);
    
    uav_deployment.flight_direction = flight_direction;
    uav_deployment.flight_speed = optimal_speed;
    uav_deployment.target_point = target_point;
    uav_deployment.flight_time = flight_distance / optimal_speed;
    
    % 设计烟幕弹投放序列
    num_smokes = randi([1, params.max_smoke_per_uav]);
    uav_deployment.smoke_deployments = design_smoke_sequence(uav_deployment, num_smokes, params, trajectory);
    
end

function best_idx = select_optimal_intercept_point(uav_pos, trajectory, params, iteration)
    % 选择最优拦截点
    
    num_points = size(trajectory.intercept_points, 1);
    scores = zeros(num_points, 1);
    
    for i = 1:num_points
        point = trajectory.intercept_points(i, :);
        
        % 距离因子 (越近越好)
        distance = norm(point - uav_pos);
        distance_score = exp(-distance / 5000);  % 归一化距离评分
        
        % 时间因子 (考虑到达时机)
        time_to_point = i / (num_points + 1) * trajectory.flight_time;
        flight_time = distance / params.uav_speed_max;
        time_score = exp(-abs(time_to_point - flight_time) / 10);
        
        % 高度因子 (适当的高度有利于烟幕扩散)
        height_score = exp(-abs(point(3) - params.real_target(3) - 100) / 100);
        
        scores(i) = distance_score * time_score * height_score;
    end
    
    % 加入随机性避免局部最优
    if iteration < params.optimization_iterations * 0.7
        randomness = 0.3;
        scores = scores + randomness * rand(size(scores));
    end
    
    [~, best_idx] = max(scores);
end

function optimal_speed = optimize_uav_speed(flight_distance, missile_flight_time, params)
    % 优化无人机飞行速度
    
    % 理想速度：在导弹到达前能够到达拦截点
    ideal_speed = flight_distance / (missile_flight_time * 0.8);  % 留出20%的时间余量
    
    % 约束到允许的速度范围
    optimal_speed = max(params.uav_speed_min, min(params.uav_speed_max, ideal_speed));
end

function smoke_deployments = design_smoke_sequence(uav_deployment, num_smokes, params, trajectory)
    % 设计烟幕弹投放序列
    
    smoke_deployments = [];
    
    for i = 1:num_smokes
        smoke = struct();
        
        % 投放时机 (考虑间隔约束)
        smoke.deploy_time = uav_deployment.flight_time * 0.8 + (i-1) * params.min_deploy_interval;
        
        % 投放位置
        deploy_distance = uav_deployment.flight_speed * smoke.deploy_time;
        smoke.deploy_position = params.uavs(uav_deployment.uav_id, :) + ...
            uav_deployment.flight_direction * deploy_distance;
        
        % 计算烟幕弹轨迹和爆炸点
        [smoke.explosion_position, smoke.explosion_time] = calculate_smoke_trajectory(...
            smoke.deploy_position, uav_deployment.target_point, params);
        
        % 计算有效遮蔽时间
        smoke.coverage_time = calculate_effective_coverage(smoke, trajectory, params);
        
        smoke_deployments = [smoke_deployments, smoke];
    end
end

function [explosion_pos, explosion_time] = calculate_smoke_trajectory(deploy_pos, target_point, params)
    % 计算烟幕弹的轨迹和最优爆炸点
    
    % 最优爆炸高度：目标上方适当位置
    optimal_explosion_height = params.real_target(3) + 80;  % 目标上方80米
    
    % 考虑烟幕下沉，计算初始爆炸位置
    explosion_pos = [target_point(1), target_point(2), optimal_explosion_height];
    
    % 计算从投放到爆炸的时间
    fall_height = deploy_pos(3) - explosion_pos(3);
    if fall_height > 0
        % 考虑初始水平速度的抛物运动
        horizontal_distance = norm(explosion_pos(1:2) - deploy_pos(1:2));
        explosion_time = (-sqrt(2*fall_height/params.gravity) + ...
            sqrt(2*fall_height/params.gravity + 2*horizontal_distance/50)) / 2;
    else
        explosion_time = 2.0;  % 默认延迟时间
    end
    
    explosion_time = max(explosion_time, 0.5);  % 最小延迟时间
end

function coverage_time = calculate_effective_coverage(smoke, trajectory, params)
    % 计算烟幕的有效遮蔽时间
    
    % 导弹通过烟幕区域的时间
    missile_pos_at_explosion = trajectory.start_pos + ...
        trajectory.direction * params.missile_speed * smoke.explosion_time;
    
    distance_to_smoke = norm(missile_pos_at_explosion - smoke.explosion_position);
    
    % 如果导弹轨迹通过烟幕有效区域
    if distance_to_smoke <= params.smoke_radius * 2  % 考虑扩散
        % 计算导弹穿越烟幕的时间
        crossing_time = 2 * params.smoke_radius / params.missile_speed;
        
        % 烟幕存在时间
        smoke_lifetime = params.smoke_duration;
        
        % 有效遮蔽时间是两者的最小值
        coverage_time = min(crossing_time, smoke_lifetime);
    else
        coverage_time = 0;
    end
    
    % 考虑烟幕浓度衰减
    coverage_time = coverage_time * exp(-distance_to_smoke / params.smoke_radius);
end

function objective = evaluate_strategy(strategy, params, trajectories)
    % 评估策略的目标函数值
    
    total_coverage = 0;
    total_cost = 0;
    
    for m = 1:length(strategy.deployments)
        missile_deployment = strategy.deployments(m);
        
        missile_coverage = 0;
        missile_cost = 0;
        
        for u = 1:length(missile_deployment.uav_deployments)
            uav_deployment = missile_deployment.uav_deployments(u);
            
            % 计算该无人机对该导弹的总遮蔽时间
            uav_coverage = 0;
            for s = 1:length(uav_deployment.smoke_deployments)
                smoke = uav_deployment.smoke_deployments(s);
                uav_coverage = uav_coverage + smoke.coverage_time;
            end
            
            missile_coverage = missile_coverage + uav_coverage;
            missile_cost = missile_cost + length(uav_deployment.smoke_deployments);
        end
        
        total_coverage = total_coverage + missile_coverage;
        total_cost = total_cost + missile_cost;
    end
    
    % 目标函数：最大化遮蔽时间，最小化成本
    objective = total_coverage - 0.1 * total_cost;  % 成本权重较小
end

function metrics = calculate_performance_metrics(strategy, params, trajectories)
    % 计算详细的性能指标
    
    metrics = struct();
    metrics.total_coverage_time = 0;
    metrics.total_smokes_used = 0;
    metrics.uav_utilization = zeros(size(params.uavs, 1), 1);
    metrics.missile_coverage = zeros(length(trajectories), 1);
    
    for m = 1:length(strategy.deployments)
        missile_deployment = strategy.deployments(m);
        
        missile_coverage = 0;
        for u = 1:length(missile_deployment.uav_deployments)
            uav_deployment = missile_deployment.uav_deployments(u);
            
            % 统计无人机使用情况
            metrics.uav_utilization(uav_deployment.uav_id) = ...
                metrics.uav_utilization(uav_deployment.uav_id) + 1;
            
            % 统计烟幕弹使用数量
            num_smokes = length(uav_deployment.smoke_deployments);
            metrics.total_smokes_used = metrics.total_smokes_used + num_smokes;
            
            % 计算遮蔽时间
            for s = 1:length(uav_deployment.smoke_deployments)
                smoke = uav_deployment.smoke_deployments(s);
                missile_coverage = missile_coverage + smoke.coverage_time;
            end
        end
        
        metrics.missile_coverage(m) = missile_coverage;
        metrics.total_coverage_time = metrics.total_coverage_time + missile_coverage;
    end
    
    metrics.average_coverage_per_missile = metrics.total_coverage_time / length(trajectories);
    metrics.efficiency = metrics.total_coverage_time / metrics.total_smokes_used;
end

function save_results_to_excel(strategy, metrics, filename)
    % 保存详细结果到Excel文件
    
    fprintf('\n正在保存结果到 %s...\n', filename);
    
    % 准备数据
    data = {};
    headers = {'导弹编号', '无人机编号', '飞行方向X', '飞行方向Y', '飞行方向Z', ...
               '飞行速度(m/s)', '目标点X', '目标点Y', '目标点Z', '飞行时间(s)', ...
               '烟幕弹序号', '投放时间(s)', '投放位置X', '投放位置Y', '投放位置Z', ...
               '爆炸位置X', '爆炸位置Y', '爆炸位置Z', '爆炸时间(s)', '遮蔽时间(s)'};
    
    data{1, 1} = headers;
    row = 2;
    
    for m = 1:length(strategy.deployments)
        missile_deployment = strategy.deployments(m);
        
        for u = 1:length(missile_deployment.uav_deployments)
            uav_deployment = missile_deployment.uav_deployments(u);
            
            for s = 1:length(uav_deployment.smoke_deployments)
                smoke = uav_deployment.smoke_deployments(s);
                
                data{row, 1} = sprintf('M%d', m);
                data{row, 2} = sprintf('FY%d', uav_deployment.uav_id);
                data{row, 3} = uav_deployment.flight_direction(1);
                data{row, 4} = uav_deployment.flight_direction(2);
                data{row, 5} = uav_deployment.flight_direction(3);
                data{row, 6} = uav_deployment.flight_speed;
                data{row, 7} = uav_deployment.target_point(1);
                data{row, 8} = uav_deployment.target_point(2);
                data{row, 9} = uav_deployment.target_point(3);
                data{row, 10} = uav_deployment.flight_time;
                data{row, 11} = s;
                data{row, 12} = smoke.deploy_time;
                data{row, 13} = smoke.deploy_position(1);
                data{row, 14} = smoke.deploy_position(2);
                data{row, 15} = smoke.deploy_position(3);
                data{row, 16} = smoke.explosion_position(1);
                data{row, 17} = smoke.explosion_position(2);
                data{row, 18} = smoke.explosion_position(3);
                data{row, 19} = smoke.explosion_time;
                data{row, 20} = smoke.coverage_time;
                
                row = row + 1;
            end
        end
    end
    
    % 添加性能指标汇总
    summary_start_row = row + 2;
    data{summary_start_row, 1} = '性能指标汇总';
    data{summary_start_row+1, 1} = '总遮蔽时间(s)';
    data{summary_start_row+1, 2} = metrics.total_coverage_time;
    data{summary_start_row+2, 1} = '总烟幕弹数量';
    data{summary_start_row+2, 2} = metrics.total_smokes_used;
    data{summary_start_row+3, 1} = '平均遮蔽时间(s)';
    data{summary_start_row+3, 2} = metrics.average_coverage_per_missile;
    data{summary_start_row+4, 1} = '效率(遮蔽时间/烟幕弹)';
    data{summary_start_row+4, 2} = metrics.efficiency;
    
    % 保存到Excel文件
    try
        % 转换为cell数组格式
        max_col = 20;
        excel_data = cell(row + 10, max_col);
        
        for i = 1:size(data, 1)
            for j = 1:size(data, 2)
                if ~isempty(data{i, j})
                    excel_data{i, j} = data{i, j};
                end
            end
        end
        
        writecell(excel_data, filename);
        fprintf('结果成功保存到 %s\n', filename);
        
    catch ME
        fprintf('保存Excel文件失败: %s\n', ME.message);
        % 备用：保存为CSV文件
        csv_filename = strrep(filename, '.xlsx', '.csv');
        writecell(excel_data, csv_filename);
        fprintf('已保存为CSV格式: %s\n', csv_filename);
    end
end

function create_comprehensive_visualization(params, strategy, metrics)
    % 创建综合可视化图表
    
    figure('Position', [50, 50, 1400, 900]);
    
    % 3D策略视图
    subplot(2, 3, [1, 2]);
    plot_3d_strategy(params, strategy);
    
    % 时间线图
    subplot(2, 3, 3);
    plot_timeline(strategy, params);
    
    % 性能分析
    subplot(2, 3, 4);
    plot_performance_metrics(metrics);
    
    % 无人机利用率
    subplot(2, 3, 5);
    plot_uav_utilization(metrics, size(params.uavs, 1));
    
    % 导弹遮蔽效果
    subplot(2, 3, 6);
    plot_missile_coverage(metrics);
    
    sgtitle('烟幕干扰弹投放策略综合分析', 'FontSize', 16, 'FontWeight', 'bold');
end

function plot_3d_strategy(params, strategy)
    % 绘制3D策略图
    
    hold on; grid on;
    
    % 绘制导弹和轨迹
    colors = ['r', 'g', 'b'];
    for i = 1:size(params.missiles, 1)
        missile_pos = params.missiles(i, :);
        target_pos = params.real_target;
        
        % 导弹位置
        plot3(missile_pos(1), missile_pos(2), missile_pos(3), 'o', ...
            'Color', colors(i), 'MarkerSize', 10, 'MarkerFaceColor', colors(i));
        
        % 导弹轨迹
        plot3([missile_pos(1), target_pos(1)], [missile_pos(2), target_pos(2)], ...
            [missile_pos(3), target_pos(3)], '--', 'Color', colors(i), 'LineWidth', 2);
        
        text(missile_pos(1), missile_pos(2), missile_pos(3)+200, ...
            sprintf('M%d', i), 'FontSize', 12, 'Color', colors(i));
    end
    
    % 绘制无人机
    for i = 1:size(params.uavs, 1)
        uav_pos = params.uavs(i, :);
        plot3(uav_pos(1), uav_pos(2), uav_pos(3), 's', ...
            'Color', 'blue', 'MarkerSize', 8, 'MarkerFaceColor', 'cyan');
        text(uav_pos(1), uav_pos(2), uav_pos(3)+100, ...
            sprintf('FY%d', i), 'FontSize', 10, 'Color', 'blue');
    end
    
    % 绘制目标
    plot3(params.real_target(1), params.real_target(2), params.real_target(3), ...
        'p', 'Color', 'green', 'MarkerSize', 15, 'MarkerFaceColor', 'green');
    text(params.real_target(1), params.real_target(2), params.real_target(3)+100, ...
        '真目标', 'FontSize', 12, 'Color', 'green');
    
    % 绘制烟幕爆炸点
    for m = 1:length(strategy.deployments)
        missile_deployment = strategy.deployments(m);
        for u = 1:length(missile_deployment.uav_deployments)
            uav_deployment = missile_deployment.uav_deployments(u);
            for s = 1:length(uav_deployment.smoke_deployments)
                smoke = uav_deployment.smoke_deployments(s);
                plot3(smoke.explosion_position(1), smoke.explosion_position(2), ...
                    smoke.explosion_position(3), 'o', 'Color', 'magenta', ...
                    'MarkerSize', 8, 'MarkerFaceColor', 'yellow');
            end
        end
    end
    
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('3D策略部署图');
    view(45, 30);
    axis equal;
end

function plot_timeline(strategy, params)
    % 绘制时间线图
    
    hold on; grid on;
    
    timeline_data = [];
    labels = {};
    
    for m = 1:length(strategy.deployments)
        missile_deployment = strategy.deployments(m);
        for u = 1:length(missile_deployment.uav_deployments)
            uav_deployment = missile_deployment.uav_deployments(u);
            for s = 1:length(uav_deployment.smoke_deployments)
                smoke = uav_deployment.smoke_deployments(s);
                timeline_data = [timeline_data; smoke.explosion_time, smoke.coverage_time];
                labels{end+1} = sprintf('M%d-FY%d-S%d', m, uav_deployment.uav_id, s);
            end
        end
    end
    
    if ~isempty(timeline_data)
        bar(timeline_data(:, 1), timeline_data(:, 2));
        xlabel('爆炸时间 (s)');
        ylabel('遮蔽时间 (s)');
        title('烟幕时间分布');
        xticks(1:length(labels));
        xticklabels(labels);
        xtickangle(45);
    end
end

function plot_performance_metrics(metrics)
    % 绘制性能指标
    
    metric_names = {'总遮蔽时间', '烟幕弹数量', '平均遮蔽时间', '效率指标'};
    metric_values = [metrics.total_coverage_time, metrics.total_smokes_used, ...
        metrics.average_coverage_per_missile, metrics.efficiency];
    
    bar(metric_values);
    set(gca, 'XTickLabel', metric_names);
    title('性能指标');
    ylabel('数值');
    xtickangle(45);
    grid on;
end

function plot_uav_utilization(metrics, num_uavs)
    % 绘制无人机利用率
    
    uav_labels = cell(num_uavs, 1);
    for i = 1:num_uavs
        uav_labels{i} = sprintf('FY%d', i);
    end
    
    bar(metrics.uav_utilization);
    set(gca, 'XTickLabel', uav_labels);
    xlabel('无人机编号');
    ylabel('参与任务数');
    title('无人机利用情况');
    grid on;
end

function plot_missile_coverage(metrics)
    % 绘制导弹遮蔽效果
    
    missile_labels = cell(length(metrics.missile_coverage), 1);
    for i = 1:length(metrics.missile_coverage)
        missile_labels{i} = sprintf('M%d', i);
    end
    
    bar(metrics.missile_coverage);
    set(gca, 'XTickLabel', missile_labels);
    xlabel('导弹编号');
    ylabel('遮蔽时间 (s)');
    title('各导弹遮蔽效果');
    grid on;
end

function print_strategy_summary(strategy, metrics)
    % 打印策略摘要
    
    fprintf('\n=== 策略执行摘要 ===\n');
    fprintf('总遮蔽时间: %.2f 秒\n', metrics.total_coverage_time);
    fprintf('使用烟幕弹总数: %d 枚\n', metrics.total_smokes_used);
    fprintf('平均每导弹遮蔽时间: %.2f 秒\n', metrics.average_coverage_per_missile);
    fprintf('效率指标: %.2f 秒/枚\n', metrics.efficiency);
    
    fprintf('\n各导弹详细策略:\n');
    for m = 1:length(strategy.deployments)
        fprintf('导弹 M%d: 遮蔽时间 %.2f 秒\n', m, metrics.missile_coverage(m));
        
        missile_deployment = strategy.deployments(m);
        for u = 1:length(missile_deployment.uav_deployments)
            uav_deployment = missile_deployment.uav_deployments(u);
            fprintf('  - 无人机 FY%d: 投放 %d 枚烟幕弹\n', ...
                uav_deployment.uav_id, length(uav_deployment.smoke_deployments));
        end
    end
end

% 执行主程序
advanced_smoke_optimization();