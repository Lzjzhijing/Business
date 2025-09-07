function test_coverage_calculation()
    % 测试修正后的覆盖计算函数
    
    clc; clear; close all;
    
    % 基本参数
    g = 9.8;
    smoke_sink_speed = 3;
    smoke_effective_radius = 10;
    smoke_effective_time = 20;
    missile_speed = 300;
    
    % 目标位置
    fake_target = [0, 0, 0];
    
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
    
    n_drones = 5;
    
    % 测试一个简单的解
    x_test = [
        100, 100, 100, 100, 100, ...  % 无人机速度
        10, 12, 14, 15, 17, 19, 20, 22, 24, 25, 27, 29, 30, 32, 34, ...  % 投放时间
        11, 13, 15, 16, 18, 20, 21, 23, 25, 26, 28, 30, 31, 33, 35   % 起爆时间
    ];
    
    fprintf('测试覆盖计算函数...\n');
    
    % 调用修正后的函数
    effective_time = calculate_effective_coverage_time_pso_fixed(x_test, drones, missiles, ...
                                                              fake_target, smoke_effective_radius, ...
                                                              smoke_effective_time, g, smoke_sink_speed, ...
                                                              missile_speed, n_drones);
    
    fprintf('计算得到的有效遮蔽时间: %.2f 秒\n', effective_time);
    
    if effective_time > 0
        fprintf('✓ 覆盖计算函数工作正常！\n');
    else
        fprintf('✗ 覆盖计算仍有问题\n');
        
        % 详细调试
        fprintf('\n开始详细调试...\n');
        debug_coverage_calculation(x_test, drones, missiles, fake_target, smoke_effective_radius, ...
                                 smoke_effective_time, g, smoke_sink_speed, missile_speed, n_drones);
    end
end

function debug_coverage_calculation(x, drones, missiles, target, smoke_effective_radius, ...
                                  smoke_effective_time, g, smoke_sink_speed, missile_speed, n_drones)
    % 详细调试覆盖计算
    
    n_smoke_total = n_drones * 3;
    drone_speeds = x(1:n_drones);
    drop_times = x(n_drones+1:n_drones+n_smoke_total);
    detonation_times = x(n_drones+n_smoke_total+1:end);
    
    fprintf('无人机速度: ');
    fprintf('%.1f ', drone_speeds);
    fprintf('\n');
    
    fprintf('投放时间: ');
    fprintf('%.1f ', drop_times);
    fprintf('\n');
    
    fprintf('起爆时间: ');
    fprintf('%.1f ', detonation_times);
    fprintf('\n');
    
    % 检查几个关键时间点
    test_times = [20, 30, 40, 50];
    
    for t = test_times
        fprintf('\n时间 t=%.1f 秒:\n', t);
        
        for missile_idx = 1:3
            missile_pos = calculate_missile_position_fixed(missiles(missile_idx, :), target, missile_speed, t);
            fprintf('  导弹%d位置: (%.1f, %.1f, %.1f)\n', missile_idx, missile_pos(1), missile_pos(2), missile_pos(3));
            
            for smoke_idx = 1:min(3, n_smoke_total)  % 只检查前3个烟幕
                drone_idx = ceil(smoke_idx / 3);
                if drop_times(smoke_idx) <= t && detonation_times(smoke_idx) <= t
                    smoke_pos = calculate_smoke_position_pso_fixed(drones(drone_idx, :), drone_speeds(drone_idx), ...
                                                               drop_times(smoke_idx), detonation_times(smoke_idx), ...
                                                               t, g, smoke_sink_speed, target);
                    distance = norm(missile_pos - smoke_pos);
                    fprintf('    烟幕%d位置: (%.1f, %.1f, %.1f), 距离: %.1f m\n', ...
                            smoke_idx, smoke_pos(1), smoke_pos(2), smoke_pos(3), distance);
                    if distance <= smoke_effective_radius
                        fprintf('    ✓ 导弹%d被烟幕%d遮蔽\n', missile_idx, smoke_idx);
                    end
                end
            end
        end
    end
end

% 包含修正后的函数
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