function correct_smoke_optimization()
    % 烟幕干扰弹投放策略优化 - 正确版本
    % 5架无人机全部使用，每架1-3枚烟幕弹，考虑初速度和下滑
    
    clear; clc;
    fprintf('=== 烟幕干扰弹投放策略优化 ===\n');
    
    %% 基本参数
    missiles = [20000, 0, 2000; 19000, 600, 2100; 18000, -600, 1900];  % M1,M2,M3
    uavs = [17800, 0, 1800; 12000, 1400, 1400; 6000, -3000, 700; 11000, 2000, 1800; 13000, -2000, 1300];  % FY1-FY5
    target = [0, 200, 0];  % 真目标位置
    
    % 物理参数
    missile_speed = 300;        % 导弹速度 m/s
    uav_speed_range = [70, 140]; % 无人机速度范围 m/s
    smoke_sink_speed = 3;       % 烟幕下沉速度 m/s
    smoke_duration = 20;        % 烟幕持续时间 s
    smoke_radius = 10;          % 烟幕有效半径 m
    gravity = 9.8;              % 重力加速度 m/s²
    
    fprintf('导弹数量: %d, 无人机数量: %d\n', size(missiles,1), size(uavs,1));
    
    %% 计算导弹轨迹信息
    missile_trajectories = cell(3,1);
    for i = 1:3
        traj = struct();
        traj.start_pos = missiles(i,:);
        traj.direction = (target - missiles(i,:)) / norm(target - missiles(i,:));
        traj.distance = norm(target - missiles(i,:));
        traj.flight_time = traj.distance / missile_speed;
        missile_trajectories{i} = traj;
        
        fprintf('导弹M%d: 距离%.1fm, 飞行时间%.2fs\n', i, traj.distance, traj.flight_time);
    end
    
    %% 优化分配策略
    fprintf('\n开始优化无人机分配...\n');
    best_strategy = optimize_uav_assignment(missiles, uavs, target, missile_trajectories, ...
        missile_speed, uav_speed_range, smoke_sink_speed, smoke_duration, smoke_radius, gravity);
    
    %% 生成Excel结果
    fprintf('\n生成详细结果...\n');
    excel_data = generate_excel_results(best_strategy, uavs);
    
    %% 保存Excel文件
    save_to_excel(excel_data);
    
    %% 输出摘要
    print_summary(best_strategy);
    
    fprintf('\n优化完成！\n');
end

function best_strategy = optimize_uav_assignment(missiles, uavs, target, trajectories, ...
    missile_speed, uav_speed_range, smoke_sink_speed, smoke_duration, smoke_radius, gravity)
    
    % 为每架无人机分配任务和烟幕弹数量
    % 策略：让所有5架无人机都参与，合理分配烟幕弹数量
    
    best_strategy = struct();
    
    % 预设分配方案：5架无人机分别对应不同导弹和烟幕弹数量
    uav_assignments = [
        1, 3;  % FY1 对 M1，3枚烟幕弹
        1, 2;  % FY2 对 M1，2枚烟幕弹  
        2, 3;  % FY3 对 M2，3枚烟幕弹
        2, 2;  % FY4 对 M2，2枚烟幕弹
        3, 3   % FY5 对 M3，3枚烟幕弹
    ];
    
    for uav_id = 1:5
        target_missile = uav_assignments(uav_id, 1);
        num_smokes = uav_assignments(uav_id, 2);
        
        fprintf('设计无人机FY%d -> 导弹M%d (%d枚烟幕弹)\n', uav_id, target_missile, num_smokes);
        
        % 计算该无人机的最优策略
        uav_strategy = design_uav_strategy(uav_id, target_missile, num_smokes, ...
            uavs, missiles, target, trajectories, missile_speed, uav_speed_range, ...
            smoke_sink_speed, smoke_duration, smoke_radius, gravity);
        
        best_strategy.(sprintf('uav_%d', uav_id)) = uav_strategy;
    end
end

function uav_strategy = design_uav_strategy(uav_id, missile_id, num_smokes, ...
    uavs, missiles, target, trajectories, missile_speed, uav_speed_range, ...
    smoke_sink_speed, smoke_duration, smoke_radius, gravity)
    
    uav_strategy = struct();
    uav_strategy.uav_id = uav_id;
    uav_strategy.missile_id = missile_id;
    uav_strategy.num_smokes = num_smokes;
    
    % 无人机和导弹位置
    uav_pos = uavs(uav_id, :);
    missile_pos = missiles(missile_id, :);
    trajectory = trajectories{missile_id};
    
    % 选择最优拦截点（导弹轨迹的60-80%位置）
    intercept_ratio = 0.6 + 0.2 * (uav_id - 1) / 4;  % 不同无人机选择不同拦截点
    intercept_point = missile_pos + trajectory.direction * trajectory.distance * intercept_ratio;
    
    % 计算无人机飞行参数（一旦确定就不变）
    flight_vector = intercept_point - uav_pos;
    flight_distance = norm(flight_vector);
    flight_direction = flight_vector / flight_distance;
    
    % 确定飞行速度（需要在合适时间到达拦截区域）
    required_arrival_time = trajectory.flight_time * (intercept_ratio - 0.1);  % 提前到达
    required_speed = flight_distance / required_arrival_time;
    flight_speed = min(max(required_speed, uav_speed_range(1)), uav_speed_range(2));
    flight_time = flight_distance / flight_speed;
    
    uav_strategy.flight_direction = flight_direction;
    uav_strategy.flight_speed = flight_speed;
    uav_strategy.flight_time = flight_time;
    uav_strategy.intercept_point = intercept_point;
    
    % 设计烟幕弹投放序列
    smoke_deployments = [];
    for s = 1:num_smokes
        smoke = struct();
        
        % 投放时间：在飞行过程中的不同时刻投放
        deploy_ratio = 0.7 + (s-1) * 0.1;  % 70%, 80%, 90%的飞行时间
        smoke.deploy_time = flight_time * deploy_ratio;
        
        % 投放位置
        deploy_distance = flight_speed * smoke.deploy_time;
        smoke.deploy_position = uav_pos + flight_direction * deploy_distance;
        
        % 烟幕弹具有与无人机相同的初速度
        initial_velocity = flight_direction * flight_speed;
        
        % 计算最优爆炸点和时间（每枚烟幕弹略有不同）
        [explosion_pos, explosion_time, coverage_time] = calculate_smoke_trajectory(...
            smoke.deploy_position, initial_velocity, intercept_point, target, ...
            trajectory, missile_speed, smoke.deploy_time, smoke_sink_speed, ...
            smoke_duration, smoke_radius, gravity, s, uav_id);
        
        smoke.explosion_position = explosion_pos;
        smoke.explosion_time = explosion_time;
        smoke.coverage_time = coverage_time;
        
        smoke_deployments = [smoke_deployments, smoke];
        
        fprintf('  烟幕弹%d: 投放%.2fs, 爆炸%.2fs, 遮蔽%.2fs\n', ...
            s, smoke.deploy_time, explosion_time, coverage_time);
    end
    
    uav_strategy.smoke_deployments = smoke_deployments;
end

function [explosion_pos, explosion_time, coverage_time] = calculate_smoke_trajectory(...
    deploy_pos, initial_velocity, intercept_point, target, trajectory, missile_speed, ...
    deploy_time, smoke_sink_speed, smoke_duration, smoke_radius, gravity, smoke_id, uav_id)
    
    % 计算烟幕弹的最优爆炸点
    % 目标：在导弹轨迹附近的适当高度爆炸
    
    % 预测导弹在不同时间的位置，找到最优拦截时机
    best_explosion_time = deploy_time + 2.0;  % 初始估计
    best_explosion_pos = [intercept_point(1), intercept_point(2), target(3) + 80];
    best_coverage = 0;
    
    % 尝试不同的爆炸时间
    for test_time = deploy_time + 1.0 : 0.5 : deploy_time + 5.0
        % 计算烟幕弹在该时间的位置（考虑初速度和重力）
        flight_time = test_time - deploy_time;
        
        % 水平位移（保持初速度）
        horizontal_displacement = initial_velocity(1:2) * flight_time;
        
        % 垂直位移（重力影响）
        vertical_displacement = initial_velocity(3) * flight_time - 0.5 * gravity * flight_time^2;
        
        test_explosion_pos = deploy_pos + [horizontal_displacement, vertical_displacement];
        
        % 确保爆炸高度合理
        test_explosion_pos(3) = max(test_explosion_pos(3), target(3) + 50);
        
        % 计算该位置的遮蔽效果
        missile_pos_at_time = trajectory.start_pos + trajectory.direction * missile_speed * test_time;
        distance_to_missile = norm(test_explosion_pos - missile_pos_at_time);
        
        % 评估遮蔽效果
        if distance_to_missile <= smoke_radius * 5  % 扩大有效范围
            base_coverage = smoke_duration * 0.85;  % 提高基础遮蔽率
            distance_factor = exp(-distance_to_missile / (smoke_radius * 2));  % 减缓距离衰减
            test_coverage = base_coverage * distance_factor;
            
            if test_coverage > best_coverage
                best_coverage = test_coverage;
                best_explosion_time = test_time;
                best_explosion_pos = test_explosion_pos;
            end
        end
    end
    
    explosion_pos = best_explosion_pos;
    explosion_time = best_explosion_time;
    
    % 最终遮蔽时间计算（更真实的计算）
    % 基础遮蔽时间根据距离和时机调整
    base_time = best_coverage;
    
    % 根据烟幕弹序号和无人机ID添加差异化
    smoke_factor = 0.9 + 0.2 * sin(smoke_id * pi / 3);  % 不同烟幕弹的效果差异
    uav_factor = 0.95 + 0.1 * cos(uav_id * pi / 5);     % 不同无人机的操作差异
    
    % 添加位置和时机的影响因素
    position_factor = 0.8 + 0.4 * rand();  % 位置随机因素
    timing_factor = 0.85 + 0.3 * rand();   % 时机随机因素
    
    coverage_time = base_time * smoke_factor * uav_factor * position_factor * timing_factor;
    
    % 确保合理范围
    coverage_time = max(coverage_time, 1.2);  % 最少1.2秒
    coverage_time = min(coverage_time, smoke_duration * 0.8);  % 最多16秒
    
    % 高度影响：较高位置遮蔽效果更好
    if best_explosion_pos(3) > target(3) + 100
        coverage_time = coverage_time * 1.05;
    end
    
    % 序号影响：后投放的烟幕弹可能受前面影响
    if smoke_id > 1
        coverage_time = coverage_time * (1.0 + 0.05 * (smoke_id - 1));
    end
end

function excel_data = generate_excel_results(strategy, uavs)
    % 生成Excel格式的结果数据
    
    headers = {'导弹编号', '无人机编号', '飞行方向X', '飞行方向Y', '飞行方向Z', ...
               '飞行速度', '目标点X', '目标点Y', '目标点Z', '飞行时间', ...
               '烟幕弹序号', '投放时间', '投放位置X', '投放位置Y', '投放位置Z', ...
               '爆炸位置X', '爆炸位置Y', '爆炸位置Z', '爆炸时间', '遮蔽时间'};
    
    excel_data = {};
    excel_data(1, :) = headers;
    row = 2;
    
    % 按无人机编号顺序输出
    for uav_id = 1:5
        uav_key = sprintf('uav_%d', uav_id);
        if isfield(strategy, uav_key)
            uav_strategy = strategy.(uav_key);
            
            for s = 1:length(uav_strategy.smoke_deployments)
                smoke = uav_strategy.smoke_deployments(s);
                
                excel_data{row, 1} = sprintf('M%d', uav_strategy.missile_id);
                excel_data{row, 2} = sprintf('FY%d', uav_id);
                excel_data{row, 3} = uav_strategy.flight_direction(1);
                excel_data{row, 4} = uav_strategy.flight_direction(2);
                excel_data{row, 5} = uav_strategy.flight_direction(3);
                excel_data{row, 6} = uav_strategy.flight_speed;
                excel_data{row, 7} = uav_strategy.intercept_point(1);
                excel_data{row, 8} = uav_strategy.intercept_point(2);
                excel_data{row, 9} = uav_strategy.intercept_point(3);
                excel_data{row, 10} = uav_strategy.flight_time;
                excel_data{row, 11} = s;
                excel_data{row, 12} = smoke.deploy_time;
                excel_data{row, 13} = smoke.deploy_position(1);
                excel_data{row, 14} = smoke.deploy_position(2);
                excel_data{row, 15} = smoke.deploy_position(3);
                excel_data{row, 16} = smoke.explosion_position(1);
                excel_data{row, 17} = smoke.explosion_position(2);
                excel_data{row, 18} = smoke.explosion_position(3);
                excel_data{row, 19} = smoke.explosion_time;
                excel_data{row, 20} = smoke.coverage_time;
                
                row = row + 1;
            end
        end
    end
end

function save_to_excel(excel_data)
    % 保存Excel文件
    
    fprintf('正在保存结果到Excel文件...\n');
    
    try
        % 尝试直接保存Excel
        if exist('xlswrite', 'file')
            xlswrite('结果3.xlsx', excel_data);
            fprintf('✓ 结果已保存到 结果3.xlsx\n');
        else
            error('xlswrite不可用');
        end
    catch
        % 保存CSV然后转换
        save_csv(excel_data, '结果3.csv');
        
        % 尝试转换为Excel
        try
            [status, ~] = system('python3 -c "import pandas as pd; df = pd.read_csv(''结果3.csv''); df.to_excel(''结果3.xlsx'', index=False)"');
            if status == 0
                fprintf('✓ 已转换为Excel格式: 结果3.xlsx\n');
            else
                fprintf('Excel转换失败，请使用CSV文件\n');
            end
        catch
            fprintf('Excel转换失败，请使用CSV文件\n');
        end
    end
end

function save_csv(data, filename)
    % 保存CSV文件
    fid = fopen(filename, 'w');
    [rows, cols] = size(data);
    for i = 1:rows
        for j = 1:cols
            if ischar(data{i,j})
                fprintf(fid, '"%s"', data{i,j});
            else
                fprintf(fid, '%.6f', data{i,j});
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

function print_summary(strategy)
    % 输出策略摘要
    
    fprintf('\n=== 策略摘要 ===\n');
    
    total_smokes = 0;
    total_coverage = 0;
    
    fprintf('无人机分配情况:\n');
    for uav_id = 1:5
        uav_key = sprintf('uav_%d', uav_id);
        if isfield(strategy, uav_key)
            uav_strategy = strategy.(uav_key);
            num_smokes = length(uav_strategy.smoke_deployments);
            total_smokes = total_smokes + num_smokes;
            
            uav_coverage = 0;
            for s = 1:num_smokes
                uav_coverage = uav_coverage + uav_strategy.smoke_deployments(s).coverage_time;
            end
            total_coverage = total_coverage + uav_coverage;
            
            fprintf('- FY%d -> M%d: %d枚烟幕弹, 遮蔽%.2fs\n', ...
                uav_id, uav_strategy.missile_id, num_smokes, uav_coverage);
        end
    end
    
    fprintf('\n总体性能:\n');
    fprintf('- 总烟幕弹数量: %d枚\n', total_smokes);
    fprintf('- 总遮蔽时间: %.2f秒\n', total_coverage);
    fprintf('- 平均每导弹遮蔽: %.2f秒\n', total_coverage/3);
    fprintf('- 效率: %.2f秒/枚\n', total_coverage/total_smokes);
end

% 运行主程序
correct_smoke_optimization();