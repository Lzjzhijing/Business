function generate_sample_result()
    % 生成问题5的示例结果文件（不依赖MATLAB优化工具箱）
    
    clc; clear;
    
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
    
    fprintf('=== 问题5：烟幕干扰弹投放策略 ===\n');
    fprintf('导弹到达假目标时间：\n');
    fprintf('M1: %.2f 秒\n', missile_times(1));
    fprintf('M2: %.2f 秒\n', missile_times(2));
    fprintf('M3: %.2f 秒\n', missile_times(3));
    
    % 使用启发式方法生成合理的解
    [drone_speeds, drop_times, detonation_times] = generate_heuristic_solution(drones, missiles, fake_target, missile_speed);
    
    % 计算有效遮蔽时间
    effective_time = calculate_effective_coverage_time_simple(drone_speeds, drop_times, detonation_times, ...
                                                            drones, missiles, fake_target, smoke_effective_radius, ...
                                                            smoke_effective_time, g, smoke_sink_speed, missile_speed);
    
    fprintf('\n启发式解的有效遮蔽时间：%.2f 秒\n', effective_time);
    
    % 生成最终结果
    generate_problem5_results_simple(drone_speeds, drop_times, detonation_times, drones, ...
                                   missiles, fake_target, smoke_effective_radius, smoke_effective_time, ...
                                   g, smoke_sink_speed, missile_speed, effective_time);
end

function [drone_speeds, drop_times, detonation_times] = generate_heuristic_solution(drones, missiles, target, missile_speed)
    % 使用启发式方法生成合理的解
    
    n_drones = 5;
    n_smoke_total = 15;
    
    % 无人机速度：根据距离目标的远近调整速度
    drone_speeds = zeros(n_drones, 1);
    for i = 1:n_drones
        distance = norm(drones(i, :) - target);
        % 距离越远，速度越快
        speed_factor = min(1.2, distance / 15000);
        drone_speeds(i) = 70 + 50 * speed_factor; % 70-120 m/s
    end
    
    % 投放时间：根据导弹到达时间安排
    drop_times = zeros(n_smoke_total, 1);
    detonation_times = zeros(n_smoke_total, 1);
    
    % 计算导弹到达时间
    missile_times = zeros(3, 1);
    for i = 1:3
        distance = norm(missiles(i, :) - target);
        missile_times(i) = distance / missile_speed;
    end
    
    % 为每架无人机安排投放时间
    for drone_idx = 1:n_drones
        for smoke_idx = 1:3
            smoke_global_idx = (drone_idx-1)*3 + smoke_idx;
            
            % 投放时间：在导弹到达前适当时间投放
            base_time = min(missile_times) - 10; % 在最早导弹到达前10秒开始投放
            drop_times(smoke_global_idx) = base_time + (drone_idx-1)*2 + (smoke_idx-1)*1;
            
            % 起爆时间：投放后0.5-2秒起爆
            detonation_times(smoke_global_idx) = drop_times(smoke_global_idx) + 0.5 + rand()*1.5;
        end
    end
    
    % 确保投放时间不为负
    drop_times = max(0, drop_times);
    detonation_times = max(drop_times + 0.5, detonation_times);
end

function effective_time = calculate_effective_coverage_time_simple(drone_speeds, drop_times, detonation_times, ...
                                                                  drones, missiles, target, smoke_effective_radius, ...
                                                                  smoke_effective_time, g, smoke_sink_speed, missile_speed)
    % 简化的有效遮蔽时间计算
    
    n_drones = 5;
    n_smoke_total = 15;
    
    % 时间步长
    dt = 0.2;
    max_time = 100;
    time_steps = 0:dt:max_time;
    
    % 计算每个时刻的遮蔽效果
    coverage_matrix = zeros(length(time_steps), 3); % 3枚导弹
    
    for t_idx = 1:length(time_steps)
        t = time_steps(t_idx);
        
        % 计算每枚导弹在时刻t的位置
        for missile_idx = 1:3
            missile_pos = calculate_missile_position_simple(missiles(missile_idx, :), target, missile_speed, t);
            
            % 检查是否被烟幕遮蔽
            is_covered = false;
            for smoke_idx = 1:n_smoke_total
                drone_idx = ceil(smoke_idx / 3);
                if drop_times(smoke_idx) <= t && detonation_times(smoke_idx) <= t
                    % 检查烟幕是否还在有效期内
                    if t <= detonation_times(smoke_idx) + smoke_effective_time
                        % 计算烟幕云团位置
                        smoke_pos = calculate_smoke_position_simple(drones(drone_idx, :), ...
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
    effective_time = 0;
    for missile_idx = 1:3
        covered_periods = find_consecutive_periods_simple(coverage_matrix(:, missile_idx));
        for period = 1:size(covered_periods, 1)
            start_time = covered_periods(period, 1) * dt;
            end_time = covered_periods(period, 2) * dt;
            effective_time = effective_time + (end_time - start_time);
        end
    end
end

function missile_pos = calculate_missile_position_simple(initial_pos, target_pos, speed, t)
    % 计算导弹在时刻t的位置
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

function smoke_pos = calculate_smoke_position_simple(drone_pos, drone_speed, drop_time, ...
                                                   detonation_time, current_time, g, sink_speed, target)
    % 计算烟幕云团在时刻t的位置
    
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

function periods = find_consecutive_periods_simple(coverage_vector)
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

function generate_problem5_results_simple(drone_speeds, drop_times, detonation_times, drones, ...
                                         missiles, target, smoke_effective_radius, smoke_effective_time, ...
                                         g, smoke_sink_speed, missile_speed, effective_time)
    % 生成问题5的最终结果并保存到Excel文件
    
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
            drop_pos = calculate_drop_position_simple(drones(drone_idx, :), drone_speeds(drone_idx), ...
                                                   drop_times(smoke_global_idx), target);
            results{row, 5} = round(drop_pos(1), 1);
            results{row, 6} = round(drop_pos(2), 1);
            results{row, 7} = round(drop_pos(3), 1);
            
            % 计算起爆位置
            detonation_pos = calculate_detonation_position_simple(drones(drone_idx, :), drone_speeds(drone_idx), ...
                                                               drop_times(smoke_global_idx), detonation_times(smoke_global_idx), ...
                                                               g, target);
            results{row, 8} = round(detonation_pos(1), 1);
            results{row, 9} = round(detonation_pos(2), 1);
            results{row, 10} = round(detonation_pos(3), 1);
            
            % 计算有效干扰时长
            interference_time = calculate_interference_time_simple(smoke_global_idx, drone_idx, drone_speeds(drone_idx), ...
                                                                  drop_times(smoke_global_idx), detonation_times(smoke_global_idx), ...
                                                                  drones, missiles, target, smoke_effective_radius, ...
                                                                  smoke_effective_time, g, smoke_sink_speed, missile_speed);
            results{row, 11} = round(interference_time, 2);
            
            % 确定干扰的导弹编号
            interfered_missiles = find_interfered_missiles_simple(smoke_global_idx, drone_idx, drone_speeds(drone_idx), ...
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
    fprintf('\n=== 问题5：烟幕干扰弹投放策略 ===\n');
    for drone_idx = 1:5
        fprintf('\n无人机 FY%d (速度: %.1f m/s):\n', drone_idx, drone_speeds(drone_idx));
        for smoke_idx = 1:3
            smoke_global_idx = (drone_idx-1)*3 + smoke_idx;
            drop_pos = calculate_drop_position_simple(drones(drone_idx, :), drone_speeds(drone_idx), ...
                                                   drop_times(smoke_global_idx), target);
            detonation_pos = calculate_detonation_position_simple(drones(drone_idx, :), drone_speeds(drone_idx), ...
                                                               drop_times(smoke_global_idx), detonation_times(smoke_global_idx), ...
                                                               g, target);
            fprintf('  烟幕干扰弹%d: 投放时间=%.2fs, 起爆时间=%.2fs\n', ...
                    smoke_idx, drop_times(smoke_global_idx), detonation_times(smoke_global_idx));
            fprintf('    投放位置: (%.1f, %.1f, %.1f)\n', drop_pos(1), drop_pos(2), drop_pos(3));
            fprintf('    起爆位置: (%.1f, %.1f, %.1f)\n', detonation_pos(1), detonation_pos(2), detonation_pos(3));
        end
    end
    
    fprintf('\n=== 优化统计 ===\n');
    fprintf('总有效遮蔽时间: %.2f 秒\n', effective_time);
    fprintf('平均无人机速度: %.1f m/s\n', mean(drone_speeds));
    fprintf('最早投放时间: %.2f 秒\n', min(drop_times));
    fprintf('最晚起爆时间: %.2f 秒\n', max(detonation_times));
end

function drop_pos = calculate_drop_position_simple(drone_pos, drone_speed, drop_time, target)
    % 计算投放位置
    drone_direction = target - drone_pos;
    drone_direction = drone_direction / norm(drone_direction);
    drop_pos = drone_pos + drone_direction * drone_speed * drop_time;
end

function detonation_pos = calculate_detonation_position_simple(drone_pos, drone_speed, drop_time, detonation_time, g, target)
    % 计算起爆位置
    drone_direction = target - drone_pos;
    drone_direction = drone_direction / norm(drone_direction);
    drop_pos = drone_pos + drone_direction * drone_speed * drop_time;
    flight_time = detonation_time - drop_time;
    detonation_pos = [drop_pos(1), drop_pos(2), drop_pos(3) - 0.5 * g * flight_time^2];
end

function interference_time = calculate_interference_time_simple(smoke_idx, drone_idx, drone_speed, drop_time, detonation_time, ...
                                                              drones, missiles, target, smoke_effective_radius, ...
                                                              smoke_effective_time, g, smoke_sink_speed, missile_speed)
    % 计算单个烟幕干扰弹的有效干扰时长
    dt = 0.2;
    max_time = 100;
    time_steps = 0:dt:max_time;
    
    total_interference = 0;
    
    for t_idx = 1:length(time_steps)
        t = time_steps(t_idx);
        
        if drop_time <= t && detonation_time <= t && t <= detonation_time + smoke_effective_time
            % 计算烟幕位置
            smoke_pos = calculate_smoke_position_simple(drones(drone_idx, :), drone_speed, drop_time, ...
                                                     detonation_time, t, g, smoke_sink_speed, target);
            
            % 检查是否干扰任何导弹
            for missile_idx = 1:3
                missile_pos = calculate_missile_position_simple(missiles(missile_idx, :), target, missile_speed, t);
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

function interfered_missiles = find_interfered_missiles_simple(smoke_idx, drone_idx, drone_speed, drop_time, detonation_time, ...
                                                             drones, missiles, target, smoke_effective_radius, ...
                                                             smoke_effective_time, g, smoke_sink_speed, missile_speed)
    % 确定被干扰的导弹编号
    dt = 0.2;
    max_time = 100;
    time_steps = 0:dt:max_time;
    
    interfered = [false, false, false];
    
    for t_idx = 1:length(time_steps)
        t = time_steps(t_idx);
        
        if drop_time <= t && detonation_time <= t && t <= detonation_time + smoke_effective_time
            % 计算烟幕位置
            smoke_pos = calculate_smoke_position_simple(drones(drone_idx, :), drone_speed, drop_time, ...
                                                     detonation_time, t, g, smoke_sink_speed, target);
            
            % 检查每枚导弹
            for missile_idx = 1:3
                missile_pos = calculate_missile_position_simple(missiles(missile_idx, :), target, missile_speed, t);
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