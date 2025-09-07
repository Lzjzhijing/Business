function pso_smoke_optimization_fixed()
    % 基于PSO算法的烟幕干扰弹投放策略优化（修正版）
    % 问题5：5架无人机对3枚来袭导弹的干扰策略
    
    clc; clear; close all;
    
    % 基本参数
    g = 9.8; % 重力加速度 m/s^2
    smoke_sink_speed = 3; % 烟幕云团下沉速度 m/s
    smoke_effective_radius = 10; % 烟幕有效遮蔽半径 m
    smoke_effective_time = 20; % 烟幕有效遮蔽时间 s
    missile_speed = 300; % 导弹速度 m/s
    
    % 目标位置
    real_target = [0, 200, 0]; % 真目标位置
    fake_target = [0, 0, 0]; % 假目标位置
    
    % 导弹初始位置
    missiles = [
        20000, 0, 2000;      % M1
        19000, 600, 2100;    % M2
        18000, -600, 1900    % M3
    ];
    
    % 无人机初始位置
    drones = [
        17800, 0, 1800;      % FY1
        12000, 1400, 1400;   % FY2
        6000, -3000, 700;    % FY3
        11000, 2000, 1800;   % FY4
        13000, -2000, 1300   % FY5
    ];
    
    % 计算导弹到达假目标的时间
    missile_times = zeros(3, 1);
    for i = 1:3
        distance = norm(missiles(i, :) - fake_target);
        missile_times(i) = distance / missile_speed;
    end
    
    fprintf('导弹到达假目标时间：\n');
    fprintf('M1: %.2f 秒\n', missile_times(1));
    fprintf('M2: %.2f 秒\n', missile_times(2));
    fprintf('M3: %.2f 秒\n', missile_times(3));
    
    % PSO算法参数
    n_drones = 5;
    n_smoke_per_drone = 3;
    n_smoke_total = n_drones * n_smoke_per_drone;
    
    % 决策变量维度：[无人机速度(5) + 投放时间(15) + 起爆时间(15) = 35维]
    dim = n_drones + 2 * n_smoke_total;
    
    % 变量边界
    lb = [repmat(70, 1, n_drones), ...      % 无人机速度下界 70 m/s
          repmat(0, 1, n_smoke_total), ...  % 投放时间下界
          repmat(0, 1, n_smoke_total)];     % 起爆时间下界
    
    ub = [repmat(140, 1, n_drones), ...     % 无人机速度上界 140 m/s
          repmat(max(missile_times), 1, n_smoke_total), ... % 投放时间上界
          repmat(max(missile_times) + 30, 1, n_smoke_total)]; % 起爆时间上界
    
    % PSO参数设置
    options = optimoptions('particleswarm', ...
        'Display', 'iter', ...
        'MaxIterations', 100, ...  % 减少迭代次数用于调试
        'SwarmSize', 50, ...       % 减少粒子数量
        'UseParallel', false, ...  % 关闭并行计算便于调试
        'FunctionTolerance', 1e-6, ...
        'MaxStallIterations', 20);
    
    % 目标函数：最大化有效遮蔽时间
    objective = @(x) -calculate_effective_coverage_time_pso_fixed(x, drones, missiles, ...
                                                           fake_target, smoke_effective_radius, ...
                                                           smoke_effective_time, g, smoke_sink_speed, ...
                                                           missile_speed, n_drones);
    
    % 非线性约束
    nonlcon = @(x) pso_constraints_fixed(x, n_drones, n_smoke_total);
    
    % 运行PSO优化
    fprintf('\n开始PSO优化...\n');
    [x_opt, fval, exitflag, output] = particleswarm(objective, dim, lb, ub, options);
    
    if exitflag > 0
        fprintf('\nPSO优化成功完成！\n');
        fprintf('最大有效遮蔽时间：%.2f 秒\n', -fval);
        fprintf('迭代次数：%d\n', output.iterations);
        fprintf('函数评估次数：%d\n', output.funccount);
        
        % 解析最优解
        drone_speeds = x_opt(1:n_drones);
        drop_times = x_opt(n_drones+1:n_drones+n_smoke_total);
        detonation_times = x_opt(n_drones+n_smoke_total+1:end);
        
        % 生成最终结果
        generate_final_results_fixed(drone_speeds, drop_times, detonation_times, drones, ...
                             missiles, fake_target, smoke_effective_radius, smoke_effective_time, ...
                             g, smoke_sink_speed, missile_speed, n_drones, -fval);
        
        % 可视化结果
        visualize_pso_results_fixed(drone_speeds, drop_times, detonation_times, drones, missiles, ...
                             real_target, fake_target, smoke_effective_radius, g, smoke_sink_speed, missile_speed);
    else
        fprintf('PSO优化失败！\n');
    end
end

function [c, ceq] = pso_constraints_fixed(x, n_drones, n_smoke_total)
    % PSO约束函数（修正版）
    drone_speeds = x(1:n_drones);
    drop_times = x(n_drones+1:n_drones+n_smoke_total);
    detonation_times = x(n_drones+n_smoke_total+1:end);
    
    c = [];
    ceq = [];
    
    % 约束1：起爆时间必须大于投放时间（至少0.1秒间隔）
    for i = 1:n_smoke_total
        c = [c; drop_times(i) - detonation_times(i) + 0.1];
    end
    
    % 约束2：同一无人机投放的烟幕干扰弹间隔至少1秒
    for drone_idx = 1:n_drones
        for smoke_idx = 1:2
            smoke1_idx = (drone_idx-1)*3 + smoke_idx;
            smoke2_idx = (drone_idx-1)*3 + smoke_idx + 1;
            if smoke2_idx <= n_smoke_total
                c = [c; 1 - abs(drop_times(smoke1_idx) - drop_times(smoke2_idx))];
            end
        end
    end
    
    % 约束3：投放时间不能为负
    for i = 1:n_smoke_total
        c = [c; -drop_times(i)];
    end
    
    % 约束4：起爆时间不能为负
    for i = 1:n_smoke_total
        c = [c; -detonation_times(i)];
    end
end

function effective_time = calculate_effective_coverage_time_pso_fixed(x, drones, missiles, ...
                                                              target, smoke_effective_radius, ...
                                                              smoke_effective_time, g, smoke_sink_speed, ...
                                                              missile_speed, n_drones)
    % 计算有效遮蔽时间（修正版）
    n_smoke_total = n_drones * 3;
    drone_speeds = x(1:n_drones);
    drop_times = x(n_drones+1:n_drones+n_smoke_total);
    detonation_times = x(n_drones+n_smoke_total+1:end);
    
    % 时间步长
    dt = 0.1;  % 减小时间步长提高精度
    max_time = 80;
    time_steps = 0:dt:max_time;
    
    % 计算每个时刻的遮蔽效果
    coverage_matrix = zeros(length(time_steps), 3); % 3枚导弹
    
    for t_idx = 1:length(time_steps)
        t = time_steps(t_idx);
        
        % 计算每枚导弹在时刻t的位置
        for missile_idx = 1:3
            missile_pos = calculate_missile_position_fixed(missiles(missile_idx, :), ...
                                                         target, missile_speed, t);
            
            % 检查是否被烟幕遮蔽
            is_covered = false;
            for smoke_idx = 1:n_smoke_total
                drone_idx = ceil(smoke_idx / 3);
                if drop_times(smoke_idx) <= t && detonation_times(smoke_idx) <= t
                    % 计算烟幕云团位置
                    smoke_pos = calculate_smoke_position_pso_fixed(drones(drone_idx, :), ...
                                                               drone_speeds(drone_idx), ...
                                                               drop_times(smoke_idx), ...
                                                               detonation_times(smoke_idx), ...
                                                               t, g, smoke_sink_speed, target);
                    
                    % 检查导弹是否在烟幕有效范围内
                    distance = norm(missile_pos - smoke_pos);
                    if distance <= smoke_effective_radius
                        is_covered = true;
                        break;
                    end
                end
            end
            
            coverage_matrix(t_idx, missile_idx) = is_covered;
        end
    end
    
    % 计算有效遮蔽时间
    effective_time = 0;
    for missile_idx = 1:3
        covered_periods = find_consecutive_periods(coverage_matrix(:, missile_idx));
        for period = 1:size(covered_periods, 1)
            start_time = covered_periods(period, 1) * dt;
            end_time = covered_periods(period, 2) * dt;
            effective_time = effective_time + (end_time - start_time);
        end
    end
end

function missile_pos = calculate_missile_position_fixed(initial_pos, target_pos, speed, t)
    % 计算导弹在时刻t的位置（修正版）
    direction = target_pos - initial_pos;
    distance_to_target = norm(direction);
    direction = direction / distance_to_target;
    
    % 导弹移动距离
    missile_distance = speed * t;
    
    % 如果导弹已经到达目标，则位置为目标位置
    if missile_distance >= distance_to_target
        missile_pos = target_pos;
    else
        missile_pos = initial_pos + direction * missile_distance;
    end
end

function smoke_pos = calculate_smoke_position_pso_fixed(drone_pos, drone_speed, drop_time, ...
                                                       detonation_time, current_time, g, sink_speed, target)
    % 计算烟幕云团在时刻t的位置（修正版）
    
    % 计算无人机移动方向（向目标方向）
    drone_direction = target - drone_pos;
    drone_direction = drone_direction / norm(drone_direction);
    
    if current_time < drop_time
        % 烟幕干扰弹还未投放
        smoke_pos = drone_pos + drone_direction * drone_speed * current_time;
    elseif current_time < detonation_time
        % 烟幕干扰弹已投放但未起爆
        flight_time = current_time - drop_time;
        if flight_time > 0
            % 投放时的无人机位置
            drop_pos = drone_pos + drone_direction * drone_speed * drop_time;
            % 考虑重力影响的抛物线运动
            x = drop_pos(1);
            y = drop_pos(2);
            z = drop_pos(3) - 0.5 * g * flight_time^2;
            smoke_pos = [x, y, z];
        else
            smoke_pos = drone_pos + drone_direction * drone_speed * drop_time;
        end
    else
        % 烟幕云团已形成，开始下沉
        explosion_time = detonation_time - drop_time;
        drop_pos = drone_pos + drone_direction * drone_speed * drop_time;
        explosion_pos = [drop_pos(1), drop_pos(2), drop_pos(3) - 0.5 * g * explosion_time^2];
        sink_time = current_time - detonation_time;
        smoke_pos = explosion_pos - [0, 0, sink_speed * sink_time];
    end
end

function periods = find_consecutive_periods(coverage_vector)
    % 找到连续的遮蔽时间段
    periods = [];
    start_idx = 0;
    
    for i = 1:length(coverage_vector)
        if coverage_vector(i) == 1 && start_idx == 0
            start_idx = i;
        elseif coverage_vector(i) == 0 && start_idx > 0
            periods = [periods; start_idx, i-1];
            start_idx = 0;
        end
    end
    
    if start_idx > 0
        periods = [periods; start_idx, length(coverage_vector)];
    end
end

function generate_final_results_fixed(drone_speeds, drop_times, detonation_times, drones, ...
                                     missiles, target, smoke_effective_radius, smoke_effective_time, ...
                                     g, smoke_sink_speed, missile_speed, n_drones, effective_time)
    % 生成最终结果并保存到Excel文件（修正版）
    
    % 创建结果表格
    results = cell(25, 8);
    
    % 表头
    headers = {'无人机编号', '烟幕干扰弹编号', '投放时间(s)', '起爆时间(s)', ...
               '投放位置X(m)', '投放位置Y(m)', '投放位置Z(m)', '备注'};
    
    for i = 1:8
        results{1, i} = headers{i};
    end
    
    % 填充数据
    row = 2;
    for drone_idx = 1:n_drones
        for smoke_idx = 1:3
            smoke_global_idx = (drone_idx-1)*3 + smoke_idx;
            
            results{row, 1} = sprintf('FY%d', drone_idx);
            results{row, 2} = smoke_idx;
            results{row, 3} = round(drop_times(smoke_global_idx), 2);
            results{row, 4} = round(detonation_times(smoke_global_idx), 2);
            
            % 计算投放位置（考虑无人机移动）
            drop_pos = calculate_drop_position_pso_fixed(drones(drone_idx, :), drone_speeds(drone_idx), ...
                                                       drop_times(smoke_global_idx), target);
            results{row, 5} = round(drop_pos(1), 1);
            results{row, 6} = round(drop_pos(2), 1);
            results{row, 7} = round(drop_pos(3), 1);
            
            results{row, 8} = '';
            
            row = row + 1;
        end
    end
    
    % 添加优化结果总结
    results{row, 1} = '=== 优化结果总结 ===';
    results{row, 2} = '';
    results{row, 3} = '';
    results{row, 4} = '';
    results{row, 5} = '';
    results{row, 6} = '';
    results{row, 7} = '';
    results{row, 8} = '';
    row = row + 1;
    
    results{row, 1} = '总有效遮蔽时间';
    results{row, 3} = sprintf('%.2f 秒', effective_time);
    results{row, 8} = 'PSO优化目标';
    row = row + 1;
    
    results{row, 1} = '平均无人机速度';
    results{row, 3} = sprintf('%.1f m/s', mean(drone_speeds));
    results{row, 8} = '统计信息';
    row = row + 1;
    
    results{row, 1} = '最早投放时间';
    results{row, 3} = sprintf('%.2f 秒', min(drop_times));
    results{row, 8} = '统计信息';
    row = row + 1;
    
    results{row, 1} = '最晚起爆时间';
    results{row, 3} = sprintf('%.2f 秒', max(detonation_times));
    results{row, 8} = '统计信息';
    
    % 保存到Excel文件
    filename = '结果3_修正版.xlsx';
    try
        writecell(results, filename, 'Sheet', 1);
        fprintf('\n结果已保存到文件：%s\n', filename);
    catch
        % 如果writecell不可用，使用xlswrite
        xlswrite(filename, results);
        fprintf('\n结果已保存到文件：%s\n', filename);
    end
    
    % 显示详细结果
    fprintf('\n=== PSO优化后的烟幕干扰弹投放策略 ===\n');
    for drone_idx = 1:n_drones
        fprintf('\n无人机 FY%d (速度: %.1f m/s):\n', drone_idx, drone_speeds(drone_idx));
        for smoke_idx = 1:3
            smoke_global_idx = (drone_idx-1)*3 + smoke_idx;
            drop_pos = calculate_drop_position_pso_fixed(drones(drone_idx, :), drone_speeds(drone_idx), ...
                                                       drop_times(smoke_global_idx), target);
            fprintf('  烟幕干扰弹%d: 投放时间=%.2fs, 起爆时间=%.2fs, 位置=(%.1f,%.1f,%.1f)\n', ...
                    smoke_idx, drop_times(smoke_global_idx), detonation_times(smoke_global_idx), ...
                    drop_pos(1), drop_pos(2), drop_pos(3));
        end
    end
    
    fprintf('\n=== 优化统计 ===\n');
    fprintf('总有效遮蔽时间: %.2f 秒\n', effective_time);
    fprintf('平均无人机速度: %.1f m/s\n', mean(drone_speeds));
    fprintf('最早投放时间: %.2f 秒\n', min(drop_times));
    fprintf('最晚起爆时间: %.2f 秒\n', max(detonation_times));
end

function drop_pos = calculate_drop_position_pso_fixed(drone_pos, drone_speed, drop_time, target)
    % 计算投放位置（考虑无人机移动）（修正版）
    drone_direction = target - drone_pos;
    drone_direction = drone_direction / norm(drone_direction);
    drop_pos = drone_pos + drone_direction * drone_speed * drop_time;
end

function visualize_pso_results_fixed(drone_speeds, drop_times, detonation_times, drones, missiles, ...
                                    real_target, fake_target, smoke_effective_radius, g, smoke_sink_speed, missile_speed)
    % 可视化PSO优化结果（修正版）
    
    figure('Position', [100, 100, 1600, 1000]);
    
    % 3D轨迹图
    subplot(2,4,1);
    hold on;
    
    % 绘制导弹轨迹
    colors = ['r', 'g', 'b'];
    for i = 1:3
        t = 0:0.1:norm(missiles(i,:))/missile_speed;
        missile_traj = zeros(length(t), 3);
        for j = 1:length(t)
            missile_direction = fake_target - missiles(i,:);
            missile_direction = missile_direction / norm(missile_direction);
            missile_traj(j,:) = missiles(i,:) + missile_direction * missile_speed * t(j);
        end
        plot3(missile_traj(:,1), missile_traj(:,2), missile_traj(:,3), colors(i), 'LineWidth', 2);
    end
    
    % 绘制无人机位置
    plot3(drones(:,1), drones(:,2), drones(:,3), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
    
    % 绘制目标
    plot3(real_target(1), real_target(2), real_target(3), 'rs', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
    plot3(fake_target(1), fake_target(2), fake_target(3), 'bs', 'MarkerSize', 12, 'MarkerFaceColor', 'b');
    
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('导弹轨迹和无人机位置');
    legend('M1轨迹', 'M2轨迹', 'M3轨迹', '无人机', '真目标', '假目标');
    grid on;
    
    % 计算并绘制遮蔽效果
    dt = 0.1;
    max_time = 80;
    time_steps = 0:dt:max_time;
    coverage_matrix = zeros(length(time_steps), 3);
    
    for t_idx = 1:length(time_steps)
        t = time_steps(t_idx);
        for missile_idx = 1:3
            missile_pos = calculate_missile_position_fixed(missiles(missile_idx, :), fake_target, missile_speed, t);
            is_covered = false;
            for smoke_idx = 1:length(drop_times)
                drone_idx = ceil(smoke_idx / 3);
                if drop_times(smoke_idx) <= t && detonation_times(smoke_idx) <= t
                    smoke_pos = calculate_smoke_position_pso_fixed(drones(drone_idx, :), drone_speeds(drone_idx), ...
                                                               drop_times(smoke_idx), detonation_times(smoke_idx), ...
                                                               t, g, smoke_sink_speed, fake_target);
                    distance = norm(missile_pos - smoke_pos);
                    if distance <= smoke_effective_radius
                        is_covered = true;
                        break;
                    end
                end
            end
            coverage_matrix(t_idx, missile_idx) = is_covered;
        end
    end
    
    % 遮蔽效果时间序列
    subplot(2,4,2);
    hold on;
    for missile_idx = 1:3
        plot(time_steps, coverage_matrix(:, missile_idx) + missile_idx, colors(missile_idx), 'LineWidth', 2);
    end
    xlabel('时间 (s)'); ylabel('导弹编号');
    title('导弹遮蔽情况');
    legend('M1', 'M2', 'M3');
    ylim([0.5, 3.5]);
    grid on;
    
    % 无人机速度分布
    subplot(2,4,3);
    bar(1:5, drone_speeds, 'FaceColor', [0.2, 0.6, 0.8]);
    xlabel('无人机编号');
    ylabel('速度 (m/s)');
    title('无人机速度分布');
    set(gca, 'XTickLabel', {'FY1', 'FY2', 'FY3', 'FY4', 'FY5'});
    grid on;
    
    % 投放时间分布
    subplot(2,4,4);
    bar(1:15, drop_times, 'FaceColor', [0.8, 0.4, 0.2]);
    xlabel('烟幕干扰弹编号');
    ylabel('投放时间 (s)');
    title('烟幕干扰弹投放时间分布');
    grid on;
    
    % 起爆时间分布
    subplot(2,4,5);
    bar(1:15, detonation_times, 'FaceColor', [0.4, 0.8, 0.2]);
    xlabel('烟幕干扰弹编号');
    ylabel('起爆时间 (s)');
    title('烟幕干扰弹起爆时间分布');
    grid on;
    
    % 总体遮蔽效果
    subplot(2,4,6);
    total_coverage = sum(coverage_matrix, 2);
    plot(time_steps, total_coverage, 'k-', 'LineWidth', 2);
    xlabel('时间 (s)');
    ylabel('被遮蔽的导弹数量');
    title('总体遮蔽效果');
    grid on;
    
    % 无人机速度vs投放时间散点图
    subplot(2,4,7);
    scatter(drone_speeds, drop_times(1:3:end), 100, 'filled');
    xlabel('无人机速度 (m/s)');
    ylabel('首次投放时间 (s)');
    title('速度vs投放时间关系');
    grid on;
    
    % 投放时间vs起爆时间散点图
    subplot(2,4,8);
    scatter(drop_times, detonation_times, 50, 'filled');
    xlabel('投放时间 (s)');
    ylabel('起爆时间 (s)');
    title('投放时间vs起爆时间关系');
    hold on;
    plot([0, max(drop_times)], [0, max(drop_times)], 'r--', 'LineWidth', 1);
    legend('数据点', 'y=x线');
    grid on;
    
    sgtitle('PSO算法优化的烟幕干扰弹投放策略结果（修正版）');
end