function heuristic_problem5_solution()
    % 问题5启发式算法解决方案
    % 使用贪心策略和局部搜索优化烟幕干扰弹投放策略
    
    clc; clear; close all;
    
    % 基本参数
    g = 9.8; % 重力加速度 m/s^2
    smoke_sink_speed = 3; % 烟幕云团下沉速度 m/s
    smoke_effective_radius = 10; % 烟幕有效遮蔽半径 m
    smoke_effective_time = 20; % 烟幕有效遮蔽时间 s
    missile_speed = 300; % 导弹速度 m/s
    
    % 目标位置
    real_target = [0, 200, 0]; % 真目标位置
    fake_target = [0, 0, 0]; % 假目标位置（导弹飞向这里）
    
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
    
    fprintf('=== 问题5：启发式算法烟幕干扰弹投放策略 ===\n');
    fprintf('导弹到达假目标时间：\n');
    fprintf('M1: %.2f 秒\n', missile_times(1));
    fprintf('M2: %.2f 秒\n', missile_times(2));
    fprintf('M3: %.2f 秒\n', missile_times(3));
    
    % 启发式算法参数
    max_iterations = 100;
    improvement_threshold = 0.1;
    
    % 初始化解
    [drone_speeds, drop_times, detonation_times] = initialize_solution(drones, missiles, fake_target, missile_speed);
    
    % 计算初始解的目标函数值
    current_objective = calculate_objective(drone_speeds, drop_times, detonation_times, ...
                                          drones, missiles, fake_target, smoke_effective_radius, ...
                                          smoke_effective_time, g, smoke_sink_speed, missile_speed);
    
    fprintf('\n初始解目标函数值: %.2f\n', current_objective);
    
    % 启发式优化主循环
    best_objective = current_objective;
    best_drone_speeds = drone_speeds;
    best_drop_times = drop_times;
    best_detonation_times = detonation_times;
    
    fprintf('\n开始启发式优化...\n');
    
    for iteration = 1:max_iterations
        % 生成邻域解
        [new_drone_speeds, new_drop_times, new_detonation_times] = generate_neighbor_solution(...
            drone_speeds, drop_times, detonation_times, drones, missiles, fake_target, missile_speed);
        
        % 计算新解的目标函数值
        new_objective = calculate_objective(new_drone_speeds, new_drop_times, new_detonation_times, ...
                                          drones, missiles, fake_target, smoke_effective_radius, ...
                                          smoke_effective_time, g, smoke_sink_speed, missile_speed);
        
        % 接受准则：如果新解更好，则接受
        if new_objective > current_objective + improvement_threshold
            drone_speeds = new_drone_speeds;
            drop_times = new_drop_times;
            detonation_times = new_detonation_times;
            current_objective = new_objective;
            
            % 更新最优解
            if current_objective > best_objective
                best_objective = current_objective;
                best_drone_speeds = drone_speeds;
                best_drop_times = drop_times;
                best_detonation_times = detonation_times;
                fprintf('迭代 %d: 找到更好解，目标函数值: %.2f\n', iteration, best_objective);
            end
        end
        
        % 每10次迭代显示进度
        if mod(iteration, 10) == 0
            fprintf('迭代 %d: 当前目标函数值: %.2f\n', iteration, current_objective);
        end
    end
    
    fprintf('\n启发式优化完成！\n');
    fprintf('最优目标函数值: %.2f\n', best_objective);
    
    % 使用最优解生成最终结果
    generate_heuristic_results(best_drone_speeds, best_drop_times, best_detonation_times, drones, ...
                              missiles, fake_target, smoke_effective_radius, smoke_effective_time, ...
                              g, smoke_sink_speed, missile_speed, best_objective);
end

function [drone_speeds, drop_times, detonation_times] = initialize_solution(drones, missiles, target, missile_speed)
    % 初始化解：使用贪心策略
    
    n_drones = 5;
    n_smoke_total = 15;
    
    % 计算导弹到达时间
    missile_times = zeros(3, 1);
    for i = 1:3
        distance = norm(missiles(i, :) - target);
        missile_times(i) = distance / missile_speed;
    end
    
    % 无人机速度：根据距离目标的远近和重要性分配
    drone_speeds = zeros(n_drones, 1);
    for i = 1:n_drones
        distance = norm(drones(i, :) - target);
        % 距离越近，速度可以相对较慢
        speed_factor = min(1.0, distance / 20000);
        drone_speeds(i) = 75 + 50 * speed_factor; % 75-125 m/s
    end
    
    % 投放时间：使用贪心策略，优先覆盖重要时间段
    drop_times = zeros(n_smoke_total, 1);
    detonation_times = zeros(n_smoke_total, 1);
    
    % 计算关键时间点
    earliest_missile_time = min(missile_times);
    latest_missile_time = max(missile_times);
    
    % 为每架无人机分配投放时间
    for drone_idx = 1:n_drones
        for smoke_idx = 1:3
            smoke_global_idx = (drone_idx-1)*3 + smoke_idx;
            
            % 投放时间：在导弹到达前适当时间投放
            base_time = earliest_missile_time - 15;
            time_offset = (drone_idx-1)*2 + (smoke_idx-1)*1.5;
            drop_times(smoke_global_idx) = base_time + time_offset;
            
            % 起爆时间：投放后适当时间起爆
            detonation_times(smoke_global_idx) = drop_times(smoke_global_idx) + 1 + (smoke_idx-1)*0.5;
        end
    end
    
    % 确保时间约束
    drop_times = max(0, drop_times);
    detonation_times = max(drop_times + 0.5, detonation_times);
    
    fprintf('\n初始解：\n');
    fprintf('无人机速度: ');
    fprintf('%.1f ', drone_speeds);
    fprintf('m/s\n');
    fprintf('投放时间范围: %.1f - %.1f 秒\n', min(drop_times), max(drop_times));
    fprintf('起爆时间范围: %.1f - %.1f 秒\n', min(detonation_times), max(detonation_times));
end

function [new_drone_speeds, new_drop_times, new_detonation_times] = generate_neighbor_solution(...
    drone_speeds, drop_times, detonation_times, drones, missiles, target, missile_speed)
    % 生成邻域解：使用多种邻域操作
    
    n_drones = 5;
    n_smoke_total = 15;
    
    % 复制当前解
    new_drone_speeds = drone_speeds;
    new_drop_times = drop_times;
    new_detonation_times = detonation_times;
    
    % 随机选择邻域操作
    operation = randi(4);
    
    switch operation
        case 1
            % 操作1：调整无人机速度
            drone_idx = randi(n_drones);
            speed_change = (rand() - 0.5) * 20; % ±10 m/s变化
            new_speed = new_drone_speeds(drone_idx) + speed_change;
            new_drone_speeds(drone_idx) = max(70, min(140, new_speed));
            
        case 2
            % 操作2：调整投放时间
            smoke_idx = randi(n_smoke_total);
            time_change = (rand() - 0.5) * 4; % ±2秒变化
            new_drop_time = new_drop_times(smoke_idx) + time_change;
            new_drop_times(smoke_idx) = max(0, new_drop_time);
            % 相应调整起爆时间
            new_detonation_times(smoke_idx) = max(new_drop_times(smoke_idx) + 0.5, ...
                                                new_detonation_times(smoke_idx) + time_change);
            
        case 3
            % 操作3：调整起爆时间
            smoke_idx = randi(n_smoke_total);
            time_change = (rand() - 0.5) * 2; % ±1秒变化
            new_detonation_time = new_detonation_times(smoke_idx) + time_change;
            new_detonation_times(smoke_idx) = max(new_drop_times(smoke_idx) + 0.5, new_detonation_time);
            
        case 4
            % 操作4：交换同一无人机的投放时间
            drone_idx = randi(n_drones);
            smoke1_idx = (drone_idx-1)*3 + randi(3);
            smoke2_idx = (drone_idx-1)*3 + randi(3);
            if smoke1_idx ~= smoke2_idx
                % 交换投放时间
                temp = new_drop_times(smoke1_idx);
                new_drop_times(smoke1_idx) = new_drop_times(smoke2_idx);
                new_drop_times(smoke2_idx) = temp;
                % 相应调整起爆时间
                new_detonation_times(smoke1_idx) = max(new_drop_times(smoke1_idx) + 0.5, new_detonation_times(smoke1_idx));
                new_detonation_times(smoke2_idx) = max(new_drop_times(smoke2_idx) + 0.5, new_detonation_times(smoke2_idx));
            end
    end
    
    % 确保约束条件
    new_drop_times = max(0, new_drop_times);
    new_detonation_times = max(new_drop_times + 0.5, new_detonation_times);
end

function objective_value = calculate_objective(drone_speeds, drop_times, detonation_times, ...
                                             drones, missiles, target, smoke_effective_radius, ...
                                             smoke_effective_time, g, smoke_sink_speed, missile_speed)
    % 计算目标函数值：有效遮蔽时间
    
    n_drones = 5;
    n_smoke_total = 15;
    
    % 时间步长
    dt = 0.5;
    max_time = 80;
    time_steps = 0:dt:max_time;
    
    % 计算每个时刻的遮蔽效果
    coverage_matrix = zeros(length(time_steps), 3); % 3枚导弹
    
    for t_idx = 1:length(time_steps)
        t = time_steps(t_idx);
        
        % 计算每枚导弹在时刻t的位置
        for missile_idx = 1:3
            missile_pos = calculate_heuristic_missile_position(missiles(missile_idx, :), target, missile_speed, t);
            
            % 检查是否被烟幕遮蔽
            is_covered = false;
            for smoke_idx = 1:n_smoke_total
                drone_idx = ceil(smoke_idx / 3);
                
                % 检查时间条件
                if drop_times(smoke_idx) <= t && detonation_times(smoke_idx) <= t
                    % 检查烟幕是否还在有效期内
                    if t <= detonation_times(smoke_idx) + smoke_effective_time
                        % 计算烟幕云团位置
                        smoke_pos = calculate_heuristic_smoke_position(drones(drone_idx, :), ...
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
            end
            
            coverage_matrix(t_idx, missile_idx) = is_covered;
        end
    end
    
    % 计算有效遮蔽时间
    objective_value = 0;
    for missile_idx = 1:3
        covered_periods = find_heuristic_consecutive_periods(coverage_matrix(:, missile_idx));
        for period = 1:size(covered_periods, 1)
            start_time = covered_periods(period, 1) * dt;
            end_time = covered_periods(period, 2) * dt;
            objective_value = objective_value + (end_time - start_time);
        end
    end
end

function missile_pos = calculate_heuristic_missile_position(initial_pos, target_pos, speed, t)
    % 计算导弹位置
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

function smoke_pos = calculate_heuristic_smoke_position(drone_pos, drone_speed, drop_time, ...
                                                      detonation_time, current_time, g, sink_speed, target)
    % 计算烟幕位置
    
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

function periods = find_heuristic_consecutive_periods(coverage_vector)
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

function generate_heuristic_results(drone_speeds, drop_times, detonation_times, drones, ...
                                   missiles, target, smoke_effective_radius, smoke_effective_time, ...
                                   g, smoke_sink_speed, missile_speed, objective_value)
    % 生成启发式算法结果文件
    
    % 创建结果表格（按照模板格式）
    results = cell(25, 12);
    
    % 表头（按照模板格式）
    headers = {'无人机编号', '无人机运动方向', '无人机运动速度(m/s)', ...
               '烟幕干扰弹编号', '烟幕干扰弹投放点的x坐标(m)', '烟幕干扰弹投放点的y坐标(m)', ...
               '烟幕干扰弹投放点的z坐标(m)', '烟幕干扰弹起爆点的x坐标(m)', ...
               '烟幕干扰弹起爆点的y坐标(m)', '烟幕干扰弹起爆点的z坐标(m)', ...
               '有效干扰时长(s)', '干扰的导弹编号'};
    
    for i = 1:12
        results{1, i} = headers{i};
    end
    
    % 填充数据
    row = 2;
    for drone_idx = 1:5
        for smoke_idx = 1:3
            smoke_global_idx = (drone_idx-1)*3 + smoke_idx;
            
            results{row, 1} = sprintf('FY%d', drone_idx);
            
            % 计算无人机运动方向（角度）
            drone_direction = target - drones(drone_idx, :);
            angle = atan2(drone_direction(2), drone_direction(1)) * 180 / pi;
            if angle < 0
                angle = angle + 360;
            end
            results{row, 2} = round(angle, 1);
            
            results{row, 3} = round(drone_speeds(drone_idx), 1);
            results{row, 4} = smoke_idx;
            
            % 计算投放位置
            drop_pos = calculate_heuristic_drop_position(drones(drone_idx, :), drone_speeds(drone_idx), ...
                                                       drop_times(smoke_global_idx), target);
            results{row, 5} = round(drop_pos(1), 1);
            results{row, 6} = round(drop_pos(2), 1);
            results{row, 7} = round(drop_pos(3), 1);
            
            % 计算起爆位置
            detonation_pos = calculate_heuristic_detonation_position(drones(drone_idx, :), drone_speeds(drone_idx), ...
                                                                   drop_times(smoke_global_idx), detonation_times(smoke_global_idx), ...
                                                                   g, target);
            results{row, 8} = round(detonation_pos(1), 1);
            results{row, 9} = round(detonation_pos(2), 1);
            results{row, 10} = round(detonation_pos(3), 1);
            
            % 计算有效干扰时长
            interference_time = calculate_heuristic_interference_time(smoke_global_idx, drone_idx, drone_speeds(drone_idx), ...
                                                                     drop_times(smoke_global_idx), detonation_times(smoke_global_idx), ...
                                                                     drones, missiles, target, smoke_effective_radius, ...
                                                                     smoke_effective_time, g, smoke_sink_speed, missile_speed);
            results{row, 11} = round(interference_time, 2);
            
            % 确定干扰的导弹编号
            interfered_missiles = find_heuristic_interfered_missiles(smoke_global_idx, drone_idx, drone_speeds(drone_idx), ...
                                                                   drop_times(smoke_global_idx), detonation_times(smoke_global_idx), ...
                                                                   drones, missiles, target, smoke_effective_radius, ...
                                                                   smoke_effective_time, g, smoke_sink_speed, missile_speed);
            results{row, 12} = interfered_missiles;
            
            row = row + 1;
        end
    end
    
    % 保存到Excel文件
    filename = 'result3.xlsx';
    try
        writecell(results, filename, 'Sheet', 1);
        fprintf('\n结果已保存到文件：%s\n', filename);
    catch
        % 如果writecell不可用，使用xlswrite
        xlswrite(filename, results);
        fprintf('\n结果已保存到文件：%s\n', filename);
    end
    
    % 显示详细结果
    fprintf('\n=== 启发式算法优化结果 ===\n');
    for drone_idx = 1:5
        fprintf('\n无人机 FY%d (速度: %.1f m/s):\n', drone_idx, drone_speeds(drone_idx));
        for smoke_idx = 1:3
            smoke_global_idx = (drone_idx-1)*3 + smoke_idx;
            drop_pos = calculate_heuristic_drop_position(drones(drone_idx, :), drone_speeds(drone_idx), ...
                                                       drop_times(smoke_global_idx), target);
            detonation_pos = calculate_heuristic_detonation_position(drones(drone_idx, :), drone_speeds(drone_idx), ...
                                                                   drop_times(smoke_global_idx), detonation_times(smoke_global_idx), ...
                                                                   g, target);
            fprintf('  烟幕干扰弹%d: 投放时间=%.2fs, 起爆时间=%.2fs\n', ...
                    smoke_idx, drop_times(smoke_global_idx), detonation_times(smoke_global_idx));
            fprintf('    投放位置: (%.1f, %.1f, %.1f)\n', drop_pos(1), drop_pos(2), drop_pos(3));
            fprintf('    起爆位置: (%.1f, %.1f, %.1f)\n', detonation_pos(1), detonation_pos(2), detonation_pos(3));
        end
    end
    
    fprintf('\n=== 优化统计 ===\n');
    fprintf('总有效遮蔽时间: %.2f 秒\n', objective_value);
    fprintf('平均无人机速度: %.1f m/s\n', mean(drone_speeds));
    fprintf('最早投放时间: %.2f 秒\n', min(drop_times));
    fprintf('最晚起爆时间: %.2f 秒\n', max(detonation_times));
end

function drop_pos = calculate_heuristic_drop_position(drone_pos, drone_speed, drop_time, target)
    % 计算投放位置
    drone_direction = target - drone_pos;
    drone_direction = drone_direction / norm(drone_direction);
    drop_pos = drone_pos + drone_direction * drone_speed * drop_time;
end

function detonation_pos = calculate_heuristic_detonation_position(drone_pos, drone_speed, drop_time, detonation_time, g, target)
    % 计算起爆位置
    drone_direction = target - drone_pos;
    drone_direction = drone_direction / norm(drone_direction);
    drop_pos = drone_pos + drone_direction * drone_speed * drop_time;
    flight_time = detonation_time - drop_time;
    detonation_pos = [drop_pos(1), drop_pos(2), drop_pos(3) - 0.5 * g * flight_time^2];
end

function interference_time = calculate_heuristic_interference_time(smoke_idx, drone_idx, drone_speed, drop_time, detonation_time, ...
                                                                  drones, missiles, target, smoke_effective_radius, ...
                                                                  smoke_effective_time, g, smoke_sink_speed, missile_speed)
    % 计算单个烟幕干扰弹的有效干扰时长
    dt = 0.5;
    max_time = 80;
    time_steps = 0:dt:max_time;
    
    total_interference = 0;
    
    for t_idx = 1:length(time_steps)
        t = time_steps(t_idx);
        
        if drop_time <= t && detonation_time <= t && t <= detonation_time + smoke_effective_time
            % 计算烟幕位置
            smoke_pos = calculate_heuristic_smoke_position(drones(drone_idx, :), drone_speed, drop_time, ...
                                                         detonation_time, t, g, smoke_sink_speed, target);
            
            % 检查是否干扰任何导弹
            for missile_idx = 1:3
                missile_pos = calculate_heuristic_missile_position(missiles(missile_idx, :), target, missile_speed, t);
                distance = norm(missile_pos - smoke_pos);
                if distance <= smoke_effective_radius
                    total_interference = total_interference + dt;
                    break; % 只要干扰一枚导弹就算有效
                end
            end
        end
    end
    
    interference_time = total_interference;
end

function interfered_missiles = find_heuristic_interfered_missiles(smoke_idx, drone_idx, drone_speed, drop_time, detonation_time, ...
                                                                 drones, missiles, target, smoke_effective_radius, ...
                                                                 smoke_effective_time, g, smoke_sink_speed, missile_speed)
    % 确定被干扰的导弹编号
    dt = 0.5;
    max_time = 80;
    time_steps = 0:dt:max_time;
    
    interfered = [false, false, false];
    
    for t_idx = 1:length(time_steps)
        t = time_steps(t_idx);
        
        if drop_time <= t && detonation_time <= t && t <= detonation_time + smoke_effective_time
            % 计算烟幕位置
            smoke_pos = calculate_heuristic_smoke_position(drones(drone_idx, :), drone_speed, drop_time, ...
                                                         detonation_time, t, g, smoke_sink_speed, target);
            
            % 检查每枚导弹
            for missile_idx = 1:3
                missile_pos = calculate_heuristic_missile_position(missiles(missile_idx, :), target, missile_speed, t);
                distance = norm(missile_pos - smoke_pos);
                if distance <= smoke_effective_radius
                    interfered(missile_idx) = true;
                end
            end
        end
    end
    
    % 生成干扰导弹编号字符串
    interfered_missiles = '';
    for i = 1:3
        if interfered(i)
            if ~isempty(interfered_missiles)
                interfered_missiles = [interfered_missiles, ','];
            end
            interfered_missiles = [interfered_missiles, sprintf('M%d', i)];
        end
    end
    
    if isempty(interfered_missiles)
        interfered_missiles = '无';
    end
end