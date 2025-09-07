function advanced_heuristic_solution()
    % 问题5高级启发式算法解决方案
    % 结合贪心策略、模拟退火和遗传算法的混合启发式方法
    
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
    
    fprintf('=== 问题5：高级启发式算法烟幕干扰弹投放策略 ===\n');
    fprintf('导弹到达假目标时间：\n');
    fprintf('M1: %.2f 秒\n', missile_times(1));
    fprintf('M2: %.2f 秒\n', missile_times(2));
    fprintf('M3: %.2f 秒\n', missile_times(3));
    
    % 算法参数
    population_size = 20;
    max_generations = 50;
    mutation_rate = 0.1;
    crossover_rate = 0.8;
    
    % 第一阶段：贪心算法生成初始种群
    fprintf('\n第一阶段：贪心算法生成初始种群...\n');
    population = generate_initial_population(population_size, drones, missiles, fake_target, missile_speed);
    
    % 第二阶段：遗传算法优化
    fprintf('\n第二阶段：遗传算法优化...\n');
    [best_solution, best_fitness] = genetic_algorithm_optimization(population, drones, missiles, fake_target, ...
                                                                  smoke_effective_radius, smoke_effective_time, g, ...
                                                                  smoke_sink_speed, missile_speed, max_generations, ...
                                                                  mutation_rate, crossover_rate);
    
    % 第三阶段：模拟退火局部搜索
    fprintf('\n第三阶段：模拟退火局部搜索...\n');
    [final_solution, final_fitness] = simulated_annealing_search(best_solution, best_fitness, drones, missiles, fake_target, ...
                                                                smoke_effective_radius, smoke_effective_time, g, ...
                                                                smoke_sink_speed, missile_speed);
    
    fprintf('\n高级启发式算法完成！\n');
    fprintf('最终目标函数值: %.2f\n', final_fitness);
    
    % 生成最终结果
    generate_advanced_results(final_solution, drones, missiles, fake_target, smoke_effective_radius, ...
                            smoke_effective_time, g, smoke_sink_speed, missile_speed, final_fitness);
end

function population = generate_initial_population(population_size, drones, missiles, target, missile_speed)
    % 使用贪心策略生成初始种群
    
    population = cell(population_size, 1);
    
    for i = 1:population_size
        % 使用不同的贪心策略生成解
        if i <= population_size/3
            % 策略1：基于距离的贪心
            [drone_speeds, drop_times, detonation_times] = greedy_by_distance(drones, missiles, target, missile_speed);
        elseif i <= 2*population_size/3
            % 策略2：基于时间的贪心
            [drone_speeds, drop_times, detonation_times] = greedy_by_time(drones, missiles, target, missile_speed);
        else
            % 策略3：随机贪心
            [drone_speeds, drop_times, detonation_times] = greedy_random(drones, missiles, target, missile_speed);
        end
        
        population{i} = struct('drone_speeds', drone_speeds, ...
                              'drop_times', drop_times, ...
                              'detonation_times', detonation_times);
    end
end

function [drone_speeds, drop_times, detonation_times] = greedy_by_distance(drones, missiles, target, missile_speed)
    % 基于距离的贪心策略
    
    n_drones = 5;
    n_smoke_total = 15;
    
    % 计算导弹到达时间
    missile_times = zeros(3, 1);
    for i = 1:3
        distance = norm(missiles(i, :) - target);
        missile_times(i) = distance / missile_speed;
    end
    
    % 无人机速度：距离越近，速度越快（抢占有利位置）
    drone_speeds = zeros(n_drones, 1);
    for i = 1:n_drones
        distance = norm(drones(i, :) - target);
        speed_factor = min(1.0, distance / 20000);
        drone_speeds(i) = 70 + 60 * speed_factor; % 70-130 m/s
    end
    
    % 投放时间：距离近的无人机先投放
    drop_times = zeros(n_smoke_total, 1);
    detonation_times = zeros(n_smoke_total, 1);
    
    % 按距离排序无人机
    distances = zeros(n_drones, 1);
    for i = 1:n_drones
        distances(i) = norm(drones(i, :) - target);
    end
    [~, sorted_indices] = sort(distances);
    
    base_time = min(missile_times) - 20;
    for rank = 1:n_drones
        drone_idx = sorted_indices(rank);
        for smoke_idx = 1:3
            smoke_global_idx = (drone_idx-1)*3 + smoke_idx;
            drop_times(smoke_global_idx) = base_time + (rank-1)*2 + (smoke_idx-1)*1;
            detonation_times(smoke_global_idx) = drop_times(smoke_global_idx) + 1 + (smoke_idx-1)*0.5;
        end
    end
    
    % 确保约束
    drop_times = max(0, drop_times);
    detonation_times = max(drop_times + 0.5, detonation_times);
end

function [drone_speeds, drop_times, detonation_times] = greedy_by_time(drones, missiles, target, missile_speed)
    % 基于时间的贪心策略
    
    n_drones = 5;
    n_smoke_total = 15;
    
    % 计算导弹到达时间
    missile_times = zeros(3, 1);
    for i = 1:3
        distance = norm(missiles(i, :) - target);
        missile_times(i) = distance / missile_speed;
    end
    
    % 无人机速度：固定中等速度
    drone_speeds = [100, 95, 105, 90, 110]; % 固定速度
    
    % 投放时间：均匀分布覆盖关键时间段
    drop_times = zeros(n_smoke_total, 1);
    detonation_times = zeros(n_smoke_total, 1);
    
    earliest_time = min(missile_times) - 25;
    latest_time = max(missile_times) - 5;
    time_span = latest_time - earliest_time;
    
    for drone_idx = 1:n_drones
        for smoke_idx = 1:3
            smoke_global_idx = (drone_idx-1)*3 + smoke_idx;
            % 均匀分布投放时间
            drop_times(smoke_global_idx) = earliest_time + (smoke_global_idx-1) * time_span / (n_smoke_total-1);
            detonation_times(smoke_global_idx) = drop_times(smoke_global_idx) + 1 + (smoke_idx-1)*0.3;
        end
    end
    
    % 确保约束
    drop_times = max(0, drop_times);
    detonation_times = max(drop_times + 0.5, detonation_times);
end

function [drone_speeds, drop_times, detonation_times] = greedy_random(drones, missiles, target, missile_speed)
    % 随机贪心策略
    
    n_drones = 5;
    n_smoke_total = 15;
    
    % 计算导弹到达时间
    missile_times = zeros(3, 1);
    for i = 1:3
        distance = norm(missiles(i, :) - target);
        missile_times(i) = distance / missile_speed;
    end
    
    % 随机无人机速度
    drone_speeds = 70 + 70 * rand(n_drones, 1); % 70-140 m/s
    
    % 随机投放时间
    drop_times = zeros(n_smoke_total, 1);
    detonation_times = zeros(n_smoke_total, 1);
    
    base_time = min(missile_times) - 20;
    for drone_idx = 1:n_drones
        for smoke_idx = 1:3
            smoke_global_idx = (drone_idx-1)*3 + smoke_idx;
            drop_times(smoke_global_idx) = base_time + rand() * 15;
            detonation_times(smoke_global_idx) = drop_times(smoke_global_idx) + 0.5 + rand() * 2;
        end
    end
    
    % 确保约束
    drop_times = max(0, drop_times);
    detonation_times = max(drop_times + 0.5, detonation_times);
end

function [best_solution, best_fitness] = genetic_algorithm_optimization(population, drones, missiles, target, ...
                                                                       smoke_effective_radius, smoke_effective_time, g, ...
                                                                       smoke_sink_speed, missile_speed, max_generations, ...
                                                                       mutation_rate, crossover_rate)
    % 遗传算法优化
    
    population_size = length(population);
    best_fitness = -inf;
    best_solution = [];
    
    for generation = 1:max_generations
        % 计算适应度
        fitness = zeros(population_size, 1);
        for i = 1:population_size
            fitness(i) = calculate_fitness(population{i}, drones, missiles, target, ...
                                         smoke_effective_radius, smoke_effective_time, g, ...
                                         smoke_sink_speed, missile_speed);
        end
        
        % 更新最优解
        [max_fitness, max_idx] = max(fitness);
        if max_fitness > best_fitness
            best_fitness = max_fitness;
            best_solution = population{max_idx};
        end
        
        % 选择、交叉、变异
        new_population = cell(population_size, 1);
        
        % 保留最优个体
        new_population{1} = best_solution;
        
        % 生成新个体
        for i = 2:population_size
            if rand() < crossover_rate
                % 交叉操作
                parent1 = tournament_selection(population, fitness);
                parent2 = tournament_selection(population, fitness);
                child = crossover(parent1, parent2);
            else
                % 复制操作
                child = tournament_selection(population, fitness);
            end
            
            % 变异操作
            if rand() < mutation_rate
                child = mutate(child, drones, missiles, target, missile_speed);
            end
            
            new_population{i} = child;
        end
        
        population = new_population;
        
        % 显示进度
        if mod(generation, 10) == 0
            fprintf('第 %d 代: 最优适应度 = %.2f\n', generation, best_fitness);
        end
    end
end

function selected = tournament_selection(population, fitness)
    % 锦标赛选择
    tournament_size = 3;
    tournament_indices = randperm(length(population), tournament_size);
    tournament_fitness = fitness(tournament_indices);
    [~, winner_idx] = max(tournament_fitness);
    selected = population{tournament_indices(winner_idx)};
end

function child = crossover(parent1, parent2)
    % 交叉操作
    child = struct();
    
    % 无人机速度交叉
    alpha = rand();
    child.drone_speeds = alpha * parent1.drone_speeds + (1-alpha) * parent2.drone_speeds;
    
    % 投放时间交叉
    crossover_point = randi([1, length(parent1.drop_times)]);
    child.drop_times = [parent1.drop_times(1:crossover_point); parent2.drop_times(crossover_point+1:end)];
    
    % 起爆时间交叉
    child.detonation_times = [parent1.detonation_times(1:crossover_point); parent2.detonation_times(crossover_point+1:end)];
    
    % 确保约束
    child.drop_times = max(0, child.drop_times);
    child.detonation_times = max(child.drop_times + 0.5, child.detonation_times);
end

function mutated = mutate(individual, drones, missiles, target, missile_speed)
    % 变异操作
    mutated = individual;
    
    % 随机选择变异类型
    mutation_type = randi(3);
    
    switch mutation_type
        case 1
            % 速度变异
            drone_idx = randi(5);
            mutated.drone_speeds(drone_idx) = 70 + 70 * rand();
            
        case 2
            % 投放时间变异
            smoke_idx = randi(15);
            time_change = (rand() - 0.5) * 10;
            mutated.drop_times(smoke_idx) = max(0, mutated.drop_times(smoke_idx) + time_change);
            mutated.detonation_times(smoke_idx) = max(mutated.drop_times(smoke_idx) + 0.5, mutated.detonation_times(smoke_idx));
            
        case 3
            % 起爆时间变异
            smoke_idx = randi(15);
            time_change = (rand() - 0.5) * 5;
            mutated.detonation_times(smoke_idx) = max(mutated.drop_times(smoke_idx) + 0.5, mutated.detonation_times(smoke_idx) + time_change);
    end
end

function [final_solution, final_fitness] = simulated_annealing_search(initial_solution, initial_fitness, ...
                                                                     drones, missiles, target, smoke_effective_radius, ...
                                                                     smoke_effective_time, g, smoke_sink_speed, missile_speed)
    % 模拟退火局部搜索
    
    current_solution = initial_solution;
    current_fitness = initial_fitness;
    best_solution = initial_solution;
    best_fitness = initial_fitness;
    
    % 退火参数
    initial_temperature = 100;
    final_temperature = 1;
    cooling_rate = 0.95;
    max_iterations = 200;
    
    temperature = initial_temperature;
    
    for iteration = 1:max_iterations
        % 生成邻域解
        neighbor_solution = generate_neighbor_solution_advanced(current_solution, drones, missiles, target, missile_speed);
        neighbor_fitness = calculate_fitness(neighbor_solution, drones, missiles, target, ...
                                           smoke_effective_radius, smoke_effective_time, g, ...
                                           smoke_sink_speed, missile_speed);
        
        % 接受准则
        delta = neighbor_fitness - current_fitness;
        if delta > 0 || rand() < exp(delta / temperature)
            current_solution = neighbor_solution;
            current_fitness = neighbor_fitness;
            
            if current_fitness > best_fitness
                best_solution = current_solution;
                best_fitness = current_fitness;
            end
        end
        
        % 降温
        temperature = temperature * cooling_rate;
        
        if mod(iteration, 50) == 0
            fprintf('模拟退火第 %d 次迭代: 当前适应度 = %.2f\n', iteration, current_fitness);
        end
    end
    
    final_solution = best_solution;
    final_fitness = best_fitness;
end

function neighbor = generate_neighbor_solution_advanced(solution, drones, missiles, target, missile_speed)
    % 生成高级邻域解
    neighbor = solution;
    
    % 随机选择操作
    operation = randi(5);
    
    switch operation
        case 1
            % 调整单个无人机速度
            drone_idx = randi(5);
            speed_change = (rand() - 0.5) * 20;
            neighbor.drone_speeds(drone_idx) = max(70, min(140, neighbor.drone_speeds(drone_idx) + speed_change));
            
        case 2
            % 调整投放时间
            smoke_idx = randi(15);
            time_change = (rand() - 0.5) * 5;
            neighbor.drop_times(smoke_idx) = max(0, neighbor.drop_times(smoke_idx) + time_change);
            neighbor.detonation_times(smoke_idx) = max(neighbor.drop_times(smoke_idx) + 0.5, neighbor.detonation_times(smoke_idx));
            
        case 3
            % 调整起爆时间
            smoke_idx = randi(15);
            time_change = (rand() - 0.5) * 3;
            neighbor.detonation_times(smoke_idx) = max(neighbor.drop_times(smoke_idx) + 0.5, neighbor.detonation_times(smoke_idx) + time_change);
            
        case 4
            % 交换同一无人机的投放时间
            drone_idx = randi(5);
            smoke1_idx = (drone_idx-1)*3 + randi(3);
            smoke2_idx = (drone_idx-1)*3 + randi(3);
            if smoke1_idx ~= smoke2_idx
                temp = neighbor.drop_times(smoke1_idx);
                neighbor.drop_times(smoke1_idx) = neighbor.drop_times(smoke2_idx);
                neighbor.drop_times(smoke2_idx) = temp;
                neighbor.detonation_times(smoke1_idx) = max(neighbor.drop_times(smoke1_idx) + 0.5, neighbor.detonation_times(smoke1_idx));
                neighbor.detonation_times(smoke2_idx) = max(neighbor.drop_times(smoke2_idx) + 0.5, neighbor.detonation_times(smoke2_idx));
            end
            
        case 5
            % 批量调整
            for i = 1:3
                smoke_idx = randi(15);
                time_change = (rand() - 0.5) * 2;
                neighbor.drop_times(smoke_idx) = max(0, neighbor.drop_times(smoke_idx) + time_change);
                neighbor.detonation_times(smoke_idx) = max(neighbor.drop_times(smoke_idx) + 0.5, neighbor.detonation_times(smoke_idx));
            end
    end
end

function fitness = calculate_fitness(solution, drones, missiles, target, smoke_effective_radius, ...
                                   smoke_effective_time, g, smoke_sink_speed, missile_speed)
    % 计算适应度（目标函数值）
    fitness = calculate_objective_advanced(solution.drone_speeds, solution.drop_times, solution.detonation_times, ...
                                         drones, missiles, target, smoke_effective_radius, ...
                                         smoke_effective_time, g, smoke_sink_speed, missile_speed);
end

function objective_value = calculate_objective_advanced(drone_speeds, drop_times, detonation_times, ...
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
            missile_pos = calculate_advanced_missile_position(missiles(missile_idx, :), target, missile_speed, t);
            
            % 检查是否被烟幕遮蔽
            is_covered = false;
            for smoke_idx = 1:n_smoke_total
                drone_idx = ceil(smoke_idx / 3);
                
                % 检查时间条件
                if drop_times(smoke_idx) <= t && detonation_times(smoke_idx) <= t
                    % 检查烟幕是否还在有效期内
                    if t <= detonation_times(smoke_idx) + smoke_effective_time
                        % 计算烟幕云团位置
                        smoke_pos = calculate_advanced_smoke_position(drones(drone_idx, :), ...
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
        covered_periods = find_advanced_consecutive_periods(coverage_matrix(:, missile_idx));
        for period = 1:size(covered_periods, 1)
            start_time = covered_periods(period, 1) * dt;
            end_time = covered_periods(period, 2) * dt;
            objective_value = objective_value + (end_time - start_time);
        end
    end
end

function missile_pos = calculate_advanced_missile_position(initial_pos, target_pos, speed, t)
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

function smoke_pos = calculate_advanced_smoke_position(drone_pos, drone_speed, drop_time, ...
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

function periods = find_advanced_consecutive_periods(coverage_vector)
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

function generate_advanced_results(solution, drones, missiles, target, smoke_effective_radius, ...
                                  smoke_effective_time, g, smoke_sink_speed, missile_speed, objective_value)
    % 生成高级启发式算法结果文件
    
    drone_speeds = solution.drone_speeds;
    drop_times = solution.drop_times;
    detonation_times = solution.detonation_times;
    
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
            drop_pos = calculate_advanced_drop_position(drones(drone_idx, :), drone_speeds(drone_idx), ...
                                                      drop_times(smoke_global_idx), target);
            results{row, 5} = round(drop_pos(1), 1);
            results{row, 6} = round(drop_pos(2), 1);
            results{row, 7} = round(drop_pos(3), 1);
            
            % 计算起爆位置
            detonation_pos = calculate_advanced_detonation_position(drones(drone_idx, :), drone_speeds(drone_idx), ...
                                                                  drop_times(smoke_global_idx), detonation_times(smoke_global_idx), ...
                                                                  g, target);
            results{row, 8} = round(detonation_pos(1), 1);
            results{row, 9} = round(detonation_pos(2), 1);
            results{row, 10} = round(detonation_pos(3), 1);
            
            % 计算有效干扰时长
            interference_time = calculate_advanced_interference_time(smoke_global_idx, drone_idx, drone_speeds(drone_idx), ...
                                                                   drop_times(smoke_global_idx), detonation_times(smoke_global_idx), ...
                                                                   drones, missiles, target, smoke_effective_radius, ...
                                                                   smoke_effective_time, g, smoke_sink_speed, missile_speed);
            results{row, 11} = round(interference_time, 2);
            
            % 确定干扰的导弹编号
            interfered_missiles = find_advanced_interfered_missiles(smoke_global_idx, drone_idx, drone_speeds(drone_idx), ...
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
    fprintf('\n=== 高级启发式算法优化结果 ===\n');
    for drone_idx = 1:5
        fprintf('\n无人机 FY%d (速度: %.1f m/s):\n', drone_idx, drone_speeds(drone_idx));
        for smoke_idx = 1:3
            smoke_global_idx = (drone_idx-1)*3 + smoke_idx;
            drop_pos = calculate_advanced_drop_position(drones(drone_idx, :), drone_speeds(drone_idx), ...
                                                      drop_times(smoke_global_idx), target);
            detonation_pos = calculate_advanced_detonation_position(drones(drone_idx, :), drone_speeds(drone_idx), ...
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

function drop_pos = calculate_advanced_drop_position(drone_pos, drone_speed, drop_time, target)
    % 计算投放位置
    drone_direction = target - drone_pos;
    drone_direction = drone_direction / norm(drone_direction);
    drop_pos = drone_pos + drone_direction * drone_speed * drop_time;
end

function detonation_pos = calculate_advanced_detonation_position(drone_pos, drone_speed, drop_time, detonation_time, g, target)
    % 计算起爆位置
    drone_direction = target - drone_pos;
    drone_direction = drone_direction / norm(drone_direction);
    drop_pos = drone_pos + drone_direction * drone_speed * drop_time;
    flight_time = detonation_time - drop_time;
    detonation_pos = [drop_pos(1), drop_pos(2), drop_pos(3) - 0.5 * g * flight_time^2];
end

function interference_time = calculate_advanced_interference_time(smoke_idx, drone_idx, drone_speed, drop_time, detonation_time, ...
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
            smoke_pos = calculate_advanced_smoke_position(drones(drone_idx, :), drone_speed, drop_time, ...
                                                         detonation_time, t, g, smoke_sink_speed, target);
            
            % 检查是否干扰任何导弹
            for missile_idx = 1:3
                missile_pos = calculate_advanced_missile_position(missiles(missile_idx, :), target, missile_speed, t);
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

function interfered_missiles = find_advanced_interfered_missiles(smoke_idx, drone_idx, drone_speed, drop_time, detonation_time, ...
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
            smoke_pos = calculate_advanced_smoke_position(drones(drone_idx, :), drone_speed, drop_time, ...
                                                         detonation_time, t, g, smoke_sink_speed, target);
            
            % 检查每枚导弹
            for missile_idx = 1:3
                missile_pos = calculate_advanced_missile_position(missiles(missile_idx, :), target, missile_speed, t);
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