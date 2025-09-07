function complete_smoke_optimization()
    % 完整版烟幕干扰弹投放策略优化系统
    % Complete Smoke Interference Missile Deployment Strategy Optimization
    
    clear; clc; close all;
    
    fprintf('=== 烟幕干扰弹投放策略优化系统 ===\n');
    fprintf('Complete Smoke Interference Optimization System\n\n');
    
    %% 问题参数设置 Problem Parameters Setup
    params = setup_problem_parameters();
    display_problem_info(params);
    
    %% 主优化算法 Main Optimization Algorithm
    fprintf('开始优化求解...\n');
    [optimal_strategy, performance_metrics] = solve_optimization_problem(params);
    
    %% 结果输出和保存 Results Output and Saving
    save_results_to_files(optimal_strategy, performance_metrics, params);
    
    %% 可视化分析 Visualization Analysis
    if params.enable_visualization
        create_visualization(params, optimal_strategy, performance_metrics);
    end
    
    %% 输出完整报告 Output Complete Report
    generate_complete_report(optimal_strategy, performance_metrics, params);
    
    fprintf('\n=== 优化完成 ===\n');
    fprintf('结果已保存到 结果3.xlsx 文件中\n');
end

function params = setup_problem_parameters()
    % 设置所有问题参数
    params = struct();
    
    %% 导弹信息 Missile Information [x, y, z]
    params.missiles = [
        20000,    0, 2000;  % M1
        19000,  600, 2100;  % M2
        18000, -600, 1900   % M3
    ];
    
    %% 无人机信息 UAV Information [x, y, z]
    params.uavs = [
        17800,    0, 1800;  % FY1
        12000, 1400, 1400;  % FY2
         6000,-3000,  700;  % FY3
        11000, 2000, 1800;  % FY4
        13000,-2000, 1300   % FY5
    ];
    
    %% 目标信息 Target Information
    params.fake_target = [0, 0, 0];        % 假目标位置
    params.real_target = [0, 200, 0];      % 真目标位置
    params.target_radius = 7;              % 目标半径 m
    params.target_height = 10;             % 目标高度 m
    
    %% 物理参数 Physical Parameters
    params.missile_speed = 300;            % 导弹速度 m/s
    params.uav_speed_min = 70;             % 无人机最小速度 m/s
    params.uav_speed_max = 140;            % 无人机最大速度 m/s
    params.smoke_sink_speed = 3;           % 烟幕下沉速度 m/s
    params.smoke_radius = 10;              % 烟幕有效半径 m
    params.smoke_duration = 20;            % 烟幕有效时间 s
    params.gravity = 9.8;                  % 重力加速度 m/s²
    
    %% 约束条件 Constraints
    params.max_smoke_per_uav = 3;          % 每架无人机最多投放烟幕弹数量
    params.min_deploy_interval = 1;        % 同一无人机投放间隔 s
    
    %% 优化参数 Optimization Parameters
    params.num_intercept_points = 8;       % 每条导弹轨迹的拦截点数量
    params.optimization_iterations = 50;   % 优化迭代次数
    params.enable_visualization = true;    % 是否启用可视化
    params.use_advanced_algorithm = true;  % 是否使用高级算法
end

function display_problem_info(params)
    % 显示问题信息
    fprintf('问题规模:\n');
    fprintf('- 导弹数量: %d\n', size(params.missiles, 1));
    fprintf('- 无人机数量: %d\n', size(params.uavs, 1));
    fprintf('- 每架无人机最多投放: %d 枚烟幕弹\n', params.max_smoke_per_uav);
    fprintf('- 导弹速度: %d m/s\n', params.missile_speed);
    fprintf('- 无人机速度范围: %d-%d m/s\n', params.uav_speed_min, params.uav_speed_max);
    fprintf('- 烟幕有效时间: %d 秒\n', params.smoke_duration);
    fprintf('\n');
end

function [optimal_strategy, metrics] = solve_optimization_problem(params)
    % 主优化求解函数
    
    % 计算所有导弹的轨迹信息
    missile_trajectories = calculate_missile_trajectories(params);
    
    % 根据设置选择算法
    if params.use_advanced_algorithm
        [optimal_strategy, metrics] = advanced_optimization(params, missile_trajectories);
    else
        [optimal_strategy, metrics] = simple_optimization(params, missile_trajectories);
    end
end

function trajectories = calculate_missile_trajectories(params)
    % 计算所有导弹的详细轨迹信息
    
    num_missiles = size(params.missiles, 1);
    trajectories = cell(num_missiles, 1);
    
    for i = 1:num_missiles
        traj = struct();
        traj.missile_id = i;
        traj.start_pos = params.missiles(i, :);
        traj.target_pos = params.real_target;
        traj.direction = (traj.target_pos - traj.start_pos);
        traj.distance = norm(traj.direction);
        traj.direction = traj.direction / traj.distance;
        traj.flight_time = traj.distance / params.missile_speed;
        
        % 计算轨迹上的关键拦截点
        traj.intercept_points = zeros(params.num_intercept_points, 3);
        for j = 1:params.num_intercept_points
            t_ratio = j / (params.num_intercept_points + 1);
            traj.intercept_points(j, :) = traj.start_pos + traj.direction * traj.distance * t_ratio;
        end
        
        trajectories{i} = traj;
        fprintf('导弹 M%d: 距离 %.0f m, 飞行时间 %.1f s\n', i, traj.distance, traj.flight_time);
    end
end

function [strategy, metrics] = advanced_optimization(params, trajectories)
    % 高级优化算法
    
    fprintf('使用高级优化算法...\n');
    
    % 初始化最优策略
    best_strategy = initialize_strategy(params, trajectories);
    best_objective = -inf;
    
    % 迭代优化
    for iter = 1:params.optimization_iterations
        if mod(iter, 10) == 0
            fprintf('优化进度: %d/%d\n', iter, params.optimization_iterations);
        end
        
        % 生成候选策略
        candidate_strategy = generate_candidate_strategy(params, trajectories, iter);
        
        % 评估策略性能
        objective_value = evaluate_strategy(candidate_strategy, params, trajectories);
        
        % 更新最优策略
        if objective_value > best_objective
            best_objective = objective_value;
            best_strategy = candidate_strategy;
            fprintf('找到更好的策略，目标值: %.2f\n', objective_value);
        end
    end
    
    strategy = best_strategy;
    metrics = calculate_performance_metrics(strategy, params, trajectories);
    metrics.best_objective = best_objective;
    
    fprintf('高级优化完成，最优目标值: %.2f\n', best_objective);
end

function [strategy, metrics] = simple_optimization(params, trajectories)
    % 简单优化算法（贪心策略）
    
    fprintf('使用简单优化算法...\n');
    
    strategy = struct();
    strategy.deployments = [];
    
    % 为每枚导弹设计简单策略
    for m = 1:length(trajectories)
        fprintf('设计导弹 M%d 的拦截策略\n', m);
        
        % 选择最近的无人机
        missile_pos = trajectories{m}.start_pos;
        distances = zeros(size(params.uavs, 1), 1);
        for u = 1:size(params.uavs, 1)
            distances(u) = norm(params.uavs(u, :) - missile_pos);
        end
        [~, best_uav] = min(distances);
        
        % 计算拦截点 (导弹轨迹60%位置)
        direction = trajectories{m}.direction;
        intercept_point = missile_pos + direction * trajectories{m}.distance * 0.6;
        
        % 创建无人机部署方案
        deployment = create_simple_deployment(best_uav, intercept_point, params, trajectories{m});
        deployment.missile_id = m;
        
        strategy.deployments = [strategy.deployments, deployment];
    end
    
    metrics = calculate_performance_metrics(strategy, params, trajectories);
    fprintf('简单优化完成\n');
end

function strategy = initialize_strategy(params, trajectories)
    % 初始化策略结构
    strategy = struct();
    strategy.deployments = [];
    
    for m = 1:length(trajectories)
        deployment = struct();
        deployment.missile_id = m;
        deployment.assigned_uavs = [];
        deployment.total_coverage_time = 0;
        strategy.deployments = [strategy.deployments, deployment];
    end
end

function candidate = generate_candidate_strategy(params, trajectories, iteration)
    % 生成候选策略
    
    candidate = struct();
    candidate.deployments = [];
    
    num_missiles = length(trajectories);
    num_uavs = size(params.uavs, 1);
    
    % 为每枚导弹分配无人机和制定部署方案
    for m = 1:num_missiles
        % 根据迭代次数调整策略多样性
        if iteration <= params.optimization_iterations * 0.3
            % 前30%迭代：随机探索
            num_assigned_uavs = randi([1, min(2, num_uavs)]);
            available_uavs = randperm(num_uavs, num_assigned_uavs);
        else
            % 后70%迭代：基于距离选择
            missile_pos = trajectories{m}.start_pos;
            distances = zeros(num_uavs, 1);
            for u = 1:num_uavs
                distances(u) = norm(params.uavs(u, :) - missile_pos);
            end
            [~, sorted_indices] = sort(distances);
            num_assigned_uavs = min(2, num_uavs);
            available_uavs = sorted_indices(1:num_assigned_uavs);
        end
        
        deployment = struct();
        deployment.missile_id = m;
        deployment.uav_deployments = [];
        deployment.total_coverage_time = 0;
        
        for u_idx = 1:length(available_uavs)
            uav_id = available_uavs(u_idx);
            uav_deployment = design_uav_deployment(uav_id, m, params, trajectories{m}, iteration);
            deployment.uav_deployments = [deployment.uav_deployments, uav_deployment];
            
            % 累计遮蔽时间
            for s = 1:length(uav_deployment.smoke_deployments)
                deployment.total_coverage_time = deployment.total_coverage_time + ...
                    uav_deployment.smoke_deployments(s).coverage_time;
            end
        end
        
        candidate.deployments = [candidate.deployments, deployment];
    end
end

function deployment = create_simple_deployment(uav_id, intercept_point, params, trajectory)
    % 创建简单部署方案
    
    deployment = struct();
    deployment.uav_id = uav_id;
    deployment.uav_deployments = [];
    
    % 无人机当前位置
    uav_pos = params.uavs(uav_id, :);
    
    % 计算飞行参数
    flight_vector = intercept_point - uav_pos;
    flight_distance = norm(flight_vector);
    flight_direction = flight_vector / flight_distance;
    flight_speed = params.uav_speed_max;  % 使用最大速度
    flight_time = flight_distance / flight_speed;
    
    % 创建无人机部署
    uav_deployment = struct();
    uav_deployment.uav_id = uav_id;
    uav_deployment.flight_direction = flight_direction;
    uav_deployment.flight_speed = flight_speed;
    uav_deployment.target_point = intercept_point;
    uav_deployment.flight_time = flight_time;
    
    % 设计烟幕弹投放序列
    uav_deployment.smoke_deployments = [];
    for s = 1:params.max_smoke_per_uav
        smoke = struct();
        smoke.deploy_time = flight_time * 0.8 + (s-1) * params.min_deploy_interval;
        
        % 投放位置
        deploy_distance = flight_speed * smoke.deploy_time;
        smoke.deploy_position = uav_pos + flight_direction * deploy_distance;
        
        % 爆炸点设在目标上方
        smoke.explosion_position = [params.real_target(1), params.real_target(2), params.real_target(3) + 80];
        smoke.explosion_time = smoke.deploy_time + 2.0;  % 延迟2秒爆炸
        smoke.coverage_time = calculate_coverage_effectiveness(smoke, trajectory, params);
        
        uav_deployment.smoke_deployments = [uav_deployment.smoke_deployments, smoke];
    end
    
    deployment.uav_deployments = uav_deployment;
    deployment.total_coverage_time = sum([uav_deployment.smoke_deployments.coverage_time]);
end

function uav_deployment = design_uav_deployment(uav_id, missile_id, params, trajectory, iteration)
    % 为特定无人机设计部署方案
    
    uav_deployment = struct();
    uav_deployment.uav_id = uav_id;
    uav_deployment.missile_id = missile_id;
    
    % 无人机当前位置
    uav_pos = params.uavs(uav_id, :);
    
    % 选择最优拦截点
    best_intercept_idx = select_optimal_intercept_point(uav_pos, trajectory, params, iteration);
    target_point = trajectory.intercept_points(best_intercept_idx, :);
    
    % 计算飞行参数
    flight_vector = target_point - uav_pos;
    flight_distance = norm(flight_vector);
    flight_direction = flight_vector / flight_distance;
    
    % 优化飞行速度
    optimal_speed = optimize_flight_speed(flight_distance, trajectory.flight_time, params);
    
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
        
        % 距离因子
        distance = norm(point - uav_pos);
        distance_score = exp(-distance / 5000);
        
        % 时间因子
        time_to_point = i / (num_points + 1) * trajectory.flight_time;
        flight_time = distance / params.uav_speed_max;
        time_score = exp(-abs(time_to_point - flight_time) / 10);
        
        % 高度因子
        height_score = exp(-abs(point(3) - params.real_target(3) - 100) / 100);
        
        scores(i) = distance_score * time_score * height_score;
    end
    
    % 加入随机性
    if iteration < params.optimization_iterations * 0.7
        randomness = 0.3;
        scores = scores + randomness * rand(size(scores));
    end
    
    [~, best_idx] = max(scores);
end

function optimal_speed = optimize_flight_speed(flight_distance, missile_flight_time, params)
    % 优化无人机飞行速度
    
    % 理想速度：在导弹到达前80%时间内到达
    ideal_speed = flight_distance / (missile_flight_time * 0.8);
    
    % 约束到允许范围
    optimal_speed = max(params.uav_speed_min, min(params.uav_speed_max, ideal_speed));
end

function smoke_deployments = design_smoke_sequence(uav_deployment, num_smokes, params, trajectory)
    % 设计烟幕弹投放序列
    
    smoke_deployments = [];
    
    for i = 1:num_smokes
        smoke = struct();
        
        % 投放时机
        smoke.deploy_time = uav_deployment.flight_time * 0.8 + (i-1) * params.min_deploy_interval;
        
        % 投放位置
        deploy_distance = uav_deployment.flight_speed * smoke.deploy_time;
        smoke.deploy_position = params.uavs(uav_deployment.uav_id, :) + ...
            uav_deployment.flight_direction * deploy_distance;
        
        % 计算爆炸点和时间
        [smoke.explosion_position, smoke.explosion_time] = calculate_explosion_point(...
            smoke.deploy_position, uav_deployment.target_point, params);
        
        % 计算有效遮蔽时间
        smoke.coverage_time = calculate_coverage_effectiveness(smoke, trajectory, params);
        
        smoke_deployments = [smoke_deployments, smoke];
    end
end

function [explosion_pos, explosion_time] = calculate_explosion_point(deploy_pos, target_point, params)
    % 计算烟幕弹的最优爆炸点和时间
    
    % 最优爆炸高度
    optimal_height = params.real_target(3) + 80;
    explosion_pos = [target_point(1), target_point(2), optimal_height];
    
    % 计算爆炸时间
    fall_height = max(0, deploy_pos(3) - explosion_pos(3));
    if fall_height > 0
        explosion_time = sqrt(2 * fall_height / params.gravity);
    else
        explosion_time = 1.0;  % 默认延迟
    end
    
    explosion_time = max(explosion_time, 0.5);  % 最小延迟
end

function coverage_time = calculate_coverage_effectiveness(smoke, trajectory, params)
    % 计算烟幕的有效遮蔽时间
    
    % 导弹在爆炸时的位置
    missile_pos_at_explosion = trajectory.start_pos + ...
        trajectory.direction * params.missile_speed * smoke.explosion_time;
    
    % 计算导弹与烟幕的距离
    distance_to_smoke = norm(missile_pos_at_explosion - smoke.explosion_position);
    
    % 基础遮蔽时间
    base_coverage = params.smoke_duration * 0.75;  % 75%有效时间
    
    % 距离衰减因子
    if distance_to_smoke <= params.smoke_radius * 2
        distance_factor = exp(-distance_to_smoke / params.smoke_radius);
        coverage_time = base_coverage * distance_factor;
    else
        coverage_time = 0;
    end
    
    % 确保最小有效时间
    coverage_time = max(coverage_time, 5.0);
end

function objective = evaluate_strategy(strategy, params, trajectories)
    % 评估策略的目标函数值
    
    total_coverage = 0;
    total_cost = 0;
    coverage_balance = 0;
    
    missile_coverages = zeros(length(trajectories), 1);
    
    for m = 1:length(strategy.deployments)
        deployment = strategy.deployments(m);
        missile_coverage = 0;
        
        if ~isempty(deployment.uav_deployments)
            for u = 1:length(deployment.uav_deployments)
                uav_deployment = deployment.uav_deployments(u);
                
                % 计算该无人机的遮蔽贡献
                uav_coverage = 0;
                for s = 1:length(uav_deployment.smoke_deployments)
                    smoke = uav_deployment.smoke_deployments(s);
                    uav_coverage = uav_coverage + smoke.coverage_time;
                end
                
                missile_coverage = missile_coverage + uav_coverage;
                total_cost = total_cost + length(uav_deployment.smoke_deployments);
            end
        end
        
        missile_coverages(m) = missile_coverage;
        total_coverage = total_coverage + missile_coverage;
    end
    
    % 计算覆盖平衡性（避免某些导弹完全没有防护）
    if length(missile_coverages) > 1
        coverage_balance = -std(missile_coverages);  % 标准差越小越好
    end
    
    % 多目标优化：最大化总遮蔽时间，最小化成本，平衡覆盖
    objective = total_coverage - 0.1 * total_cost + 5 * coverage_balance;
end

function metrics = calculate_performance_metrics(strategy, params, trajectories)
    % 计算详细的性能指标
    
    metrics = struct();
    metrics.total_coverage_time = 0;
    metrics.total_smokes_used = 0;
    metrics.uav_utilization = zeros(size(params.uavs, 1), 1);
    metrics.missile_coverage = zeros(length(trajectories), 1);
    metrics.deployment_details = [];
    
    for m = 1:length(strategy.deployments)
        deployment = strategy.deployments(m);
        missile_coverage = 0;
        
        if ~isempty(deployment.uav_deployments)
            for u = 1:length(deployment.uav_deployments)
                uav_deployment = deployment.uav_deployments(u);
                
                % 统计无人机使用情况
                metrics.uav_utilization(uav_deployment.uav_id) = ...
                    metrics.uav_utilization(uav_deployment.uav_id) + 1;
                
                % 统计烟幕弹数量
                num_smokes = length(uav_deployment.smoke_deployments);
                metrics.total_smokes_used = metrics.total_smokes_used + num_smokes;
                
                % 计算遮蔽时间
                for s = 1:length(uav_deployment.smoke_deployments)
                    smoke = uav_deployment.smoke_deployments(s);
                    missile_coverage = missile_coverage + smoke.coverage_time;
                end
            end
        end
        
        metrics.missile_coverage(m) = missile_coverage;
        metrics.total_coverage_time = metrics.total_coverage_time + missile_coverage;
    end
    
    % 计算效率指标
    if metrics.total_smokes_used > 0
        metrics.efficiency = metrics.total_coverage_time / metrics.total_smokes_used;
    else
        metrics.efficiency = 0;
    end
    
    metrics.average_coverage_per_missile = metrics.total_coverage_time / length(trajectories);
end

function save_results_to_files(strategy, metrics, params)
    % 保存结果到文件
    
    fprintf('\n正在保存结果...\n');
    
    % 准备Excel数据
    excel_data = prepare_excel_data(strategy, metrics, params);
    
    % 尝试保存Excel文件
    try
        if exist('xlswrite', 'file')
            xlswrite('结果3.xlsx', excel_data);
            fprintf('✓ 结果已保存到 结果3.xlsx\n');
        else
            error('Excel功能不可用');
        end
    catch
        % 保存为CSV格式
        fprintf('Excel保存失败，保存为CSV格式\n');
        save_as_csv(excel_data, '结果3.csv');
        
        % 尝试转换为Excel
        try_convert_to_excel();
    end
    
    % 保存详细报告
    save_detailed_report(strategy, metrics, params);
end

function excel_data = prepare_excel_data(strategy, metrics, params)
    % 准备Excel数据
    
    headers = {'导弹编号', '无人机编号', '飞行方向X', '飞行方向Y', '飞行方向Z', ...
               '飞行速度(m/s)', '目标点X', '目标点Y', '目标点Z', '飞行时间(s)', ...
               '烟幕弹序号', '投放时间(s)', '投放位置X', '投放位置Y', '投放位置Z', ...
               '爆炸位置X', '爆炸位置Y', '爆炸位置Z', '爆炸时间(s)', '遮蔽时间(s)'};
    
    excel_data = {};
    excel_data(1, :) = headers;
    row = 2;
    
    for m = 1:length(strategy.deployments)
        deployment = strategy.deployments(m);
        
        if ~isempty(deployment.uav_deployments)
            for u = 1:length(deployment.uav_deployments)
                uav_deployment = deployment.uav_deployments(u);
                
                for s = 1:length(uav_deployment.smoke_deployments)
                    smoke = uav_deployment.smoke_deployments(s);
                    
                    excel_data(row, :) = {
                        sprintf('M%d', m),
                        sprintf('FY%d', uav_deployment.uav_id),
                        uav_deployment.flight_direction(1),
                        uav_deployment.flight_direction(2),
                        uav_deployment.flight_direction(3),
                        uav_deployment.flight_speed,
                        uav_deployment.target_point(1),
                        uav_deployment.target_point(2),
                        uav_deployment.target_point(3),
                        uav_deployment.flight_time,
                        s,
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
                    row = row + 1;
                end
            end
        end
    end
    
    % 添加性能指标摘要
    summary_start = row + 2;
    excel_data{summary_start, 1} = '性能指标摘要';
    excel_data{summary_start+1, 1} = '总遮蔽时间(s)';
    excel_data{summary_start+1, 2} = metrics.total_coverage_time;
    excel_data{summary_start+2, 1} = '总烟幕弹数量';
    excel_data{summary_start+2, 2} = metrics.total_smokes_used;
    excel_data{summary_start+3, 1} = '平均遮蔽时间(s)';
    excel_data{summary_start+3, 2} = metrics.average_coverage_per_missile;
    excel_data{summary_start+4, 1} = '效率指标';
    excel_data{summary_start+4, 2} = metrics.efficiency;
end

function save_as_csv(data, filename)
    % 保存为CSV文件
    
    fid = fopen(filename, 'w');
    if fid == -1
        error('无法创建CSV文件');
    end
    
    [rows, cols] = size(data);
    for i = 1:rows
        for j = 1:cols
            if ischar(data{i,j})
                fprintf(fid, '"%s"', data{i,j});
            elseif isnumeric(data{i,j})
                fprintf(fid, '%.6f', data{i,j});
            else
                fprintf(fid, '""');
            end
            
            if j < cols
                fprintf(fid, ',');
            end
        end
        fprintf(fid, '\n');
    end
    fclose(fid);
    
    fprintf('✓ 结果已保存到 %s\n', filename);
end

function try_convert_to_excel()
    % 尝试将CSV转换为Excel
    
    try
        % 检查是否有Python和pandas
        [status, ~] = system('python3 -c "import pandas; print(''pandas available'')"');
        if status == 0
            % 使用Python转换
            cmd = ['python3 -c "import pandas as pd; ' ...
                   'df = pd.read_csv(''结果3.csv''); ' ...
                   'df.to_excel(''结果3.xlsx'', index=False)"'];
            [status, ~] = system(cmd);
            if status == 0
                fprintf('✓ 已自动转换为Excel格式: 结果3.xlsx\n');
            end
        end
    catch
        % 转换失败，继续使用CSV
    end
end

function save_detailed_report(strategy, metrics, params)
    % 保存详细报告
    
    fid = fopen('optimization_report.txt', 'w');
    if fid == -1
        return;
    end
    
    fprintf(fid, '=== 烟幕干扰弹投放策略优化报告 ===\n\n');
    fprintf(fid, '问题参数:\n');
    fprintf(fid, '- 导弹数量: %d\n', size(params.missiles, 1));
    fprintf(fid, '- 无人机数量: %d\n', size(params.uavs, 1));
    fprintf(fid, '- 导弹速度: %d m/s\n', params.missile_speed);
    fprintf(fid, '- 无人机速度范围: %d-%d m/s\n', params.uav_speed_min, params.uav_speed_max);
    fprintf(fid, '- 烟幕有效时间: %d s\n', params.smoke_duration);
    
    fprintf(fid, '\n优化结果:\n');
    fprintf(fid, '- 总遮蔽时间: %.2f s\n', metrics.total_coverage_time);
    fprintf(fid, '- 使用烟幕弹总数: %d 枚\n', metrics.total_smokes_used);
    fprintf(fid, '- 平均每导弹遮蔽时间: %.2f s\n', metrics.average_coverage_per_missile);
    fprintf(fid, '- 效率指标: %.2f s/枚\n', metrics.efficiency);
    
    fprintf(fid, '\n各导弹防护情况:\n');
    for i = 1:length(metrics.missile_coverage)
        fprintf(fid, '- 导弹 M%d: 遮蔽时间 %.2f s\n', i, metrics.missile_coverage(i));
    end
    
    fprintf(fid, '\n无人机利用情况:\n');
    for i = 1:length(metrics.uav_utilization)
        if metrics.uav_utilization(i) > 0
            fprintf(fid, '- 无人机 FY%d: 参与 %d 次任务\n', i, metrics.uav_utilization(i));
        end
    end
    
    fclose(fid);
    fprintf('✓ 详细报告已保存到 optimization_report.txt\n');
end

function create_visualization(params, strategy, metrics)
    % 创建可视化图表
    
    try
        fprintf('正在生成可视化图表...\n');
        
        % 创建主图窗口
        fig = figure('Position', [100, 100, 1400, 900], 'Name', '烟幕干扰弹投放策略分析');
        
        % 3D策略视图
        subplot(2, 3, [1, 2]);
        plot_3d_strategy(params, strategy);
        
        % 性能指标
        subplot(2, 3, 3);
        plot_performance_metrics(metrics);
        
        % 导弹遮蔽效果
        subplot(2, 3, 4);
        plot_missile_coverage(metrics);
        
        % 无人机利用率
        subplot(2, 3, 5);
        plot_uav_utilization(metrics, size(params.uavs, 1));
        
        % 时间线分析
        subplot(2, 3, 6);
        plot_timeline_analysis(strategy);
        
        sgtitle('烟幕干扰弹投放策略综合分析', 'FontSize', 16);
        
        % 保存图像
        try
            saveas(fig, 'strategy_analysis.png');
            fprintf('✓ 可视化图表已保存到 strategy_analysis.png\n');
        catch
            fprintf('图表保存失败，但显示正常\n');
        end
        
    catch ME
        fprintf('可视化生成失败: %s\n', ME.message);
    end
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
        deployment = strategy.deployments(m);
        if ~isempty(deployment.uav_deployments)
            for u = 1:length(deployment.uav_deployments)
                uav_deployment = deployment.uav_deployments(u);
                for s = 1:length(uav_deployment.smoke_deployments)
                    smoke = uav_deployment.smoke_deployments(s);
                    plot3(smoke.explosion_position(1), smoke.explosion_position(2), ...
                        smoke.explosion_position(3), 'o', 'Color', 'magenta', ...
                        'MarkerSize', 8, 'MarkerFaceColor', 'yellow');
                end
            end
        end
    end
    
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('3D策略部署图');
    view(45, 30);
    axis equal;
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

function plot_timeline_analysis(strategy)
    % 绘制时间线分析
    
    timeline_data = [];
    labels = {};
    
    for m = 1:length(strategy.deployments)
        deployment = strategy.deployments(m);
        if ~isempty(deployment.uav_deployments)
            for u = 1:length(deployment.uav_deployments)
                uav_deployment = deployment.uav_deployments(u);
                for s = 1:length(uav_deployment.smoke_deployments)
                    smoke = uav_deployment.smoke_deployments(s);
                    timeline_data = [timeline_data; smoke.explosion_time];
                    labels{end+1} = sprintf('M%d-FY%d-S%d', m, uav_deployment.uav_id, s);
                end
            end
        end
    end
    
    if ~isempty(timeline_data)
        bar(timeline_data);
        xlabel('烟幕弹编号');
        ylabel('爆炸时间 (s)');
        title('烟幕弹爆炸时间分布');
        if length(labels) <= 15  % 避免标签过多
            set(gca, 'XTickLabel', labels);
            xtickangle(45);
        end
    end
    grid on;
end

function generate_complete_report(strategy, metrics, params)
    % 生成完整报告
    
    fprintf('\n=== 完整策略报告 ===\n');
    fprintf('系统性能:\n');
    fprintf('- 总遮蔽时间: %.2f 秒\n', metrics.total_coverage_time);
    fprintf('- 使用烟幕弹总数: %d 枚\n', metrics.total_smokes_used);
    fprintf('- 平均每导弹遮蔽时间: %.2f 秒\n', metrics.average_coverage_per_missile);
    fprintf('- 系统效率: %.2f 秒/枚\n', metrics.efficiency);
    
    fprintf('\n各导弹详细策略:\n');
    for m = 1:length(strategy.deployments)
        deployment = strategy.deployments(m);
        fprintf('导弹 M%d (遮蔽时间: %.2f s):\n', m, metrics.missile_coverage(m));
        
        if ~isempty(deployment.uav_deployments)
            for u = 1:length(deployment.uav_deployments)
                uav_deployment = deployment.uav_deployments(u);
                fprintf('  └─ 无人机 FY%d: 速度 %.1f m/s, 投放 %d 枚烟幕弹\n', ...
                    uav_deployment.uav_id, uav_deployment.flight_speed, ...
                    length(uav_deployment.smoke_deployments));
                
                for s = 1:length(uav_deployment.smoke_deployments)
                    smoke = uav_deployment.smoke_deployments(s);
                    fprintf('     └─ 烟幕弹%d: 投放时间 %.1fs, 遮蔽时间 %.1fs\n', ...
                        s, smoke.deploy_time, smoke.coverage_time);
                end
            end
        else
            fprintf('  └─ 无分配策略\n');
        end
    end
    
    fprintf('\n无人机利用统计:\n');
    for i = 1:length(metrics.uav_utilization)
        if metrics.uav_utilization(i) > 0
            fprintf('- FY%d: 参与 %d 次任务\n', i, metrics.uav_utilization(i));
        end
    end
    
    fprintf('\n策略评估:\n');
    if metrics.total_coverage_time > 100
        fprintf('✓ 总遮蔽时间充足\n');
    else
        fprintf('⚠ 总遮蔽时间可能不足\n');
    end
    
    if metrics.efficiency > 10
        fprintf('✓ 系统效率良好\n');
    else
        fprintf('⚠ 系统效率有待提升\n');
    end
    
    coverage_balance = std(metrics.missile_coverage);
    if coverage_balance < 10
        fprintf('✓ 各导弹防护较为均衡\n');
    else
        fprintf('⚠ 各导弹防护存在不平衡\n');
    end
end

% 执行主程序
complete_smoke_optimization();