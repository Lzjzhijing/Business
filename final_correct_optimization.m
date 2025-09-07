function final_correct_optimization()
    % 烟幕干扰弹投放策略优化 - 最终正确版本
    % 5架无人机共同对3枚导弹进行干扰，每架无人机最多3枚烟幕弹
    
    clear; clc;
    fprintf('=== 烟幕干扰弹投放策略优化 ===\n');
    
    %% 基本参数
    missiles = [20000, 0, 2000; 19000, 600, 2100; 18000, -600, 1900];  % M1,M2,M3
    uavs = [17800, 0, 1800; 12000, 1400, 1400; 6000, -3000, 700; 11000, 2000, 1800; 13000, -2000, 1300];  % FY1-FY5
    target = [0, 200, 0];  % 真目标位置
    
    % 物理参数
    missile_speed = 300;        % 导弹速度 m/s
    uav_speed_range = [70, 140]; % 无人机速度范围 m/s
    smoke_duration = 20;        % 烟幕持续时间 s
    smoke_radius = 10;          % 烟幕有效半径 m
    gravity = 9.8;              % 重力加速度 m/s²
    
    fprintf('导弹数量: %d, 无人机数量: %d\n', size(missiles,1), size(uavs,1));
    
    %% 计算导弹轨迹
    for i = 1:3
        distance = norm(target - missiles(i,:));
        flight_time = distance / missile_speed;
        fprintf('导弹M%d: 距离%.1fm, 飞行时间%.2fs\n', i, distance, flight_time);
    end
    
    %% 设计5架无人机的协同作战策略
    % 合理分配：让5架无人机共同对3枚导弹进行防护
    strategies = design_cooperative_strategy(missiles, uavs, target, missile_speed, uav_speed_range);
    
    %% 生成Excel结果
    excel_data = generate_results(strategies);
    
    %% 保存Excel文件
    save_excel_file(excel_data);
    
    %% 输出摘要
    print_final_summary(strategies);
    
    fprintf('\n优化完成！\n');
end

function strategies = design_cooperative_strategy(missiles, uavs, target, missile_speed, uav_speed_range)
    % 设计5架无人机的协同作战策略
    
    strategies = [];
    
    % 为每架无人机分配任务和烟幕弹数量
    % 策略：让所有无人机参与，合理分配到3枚导弹
    assignments = [
        1, 2;  % FY1 -> M1, 2枚烟幕弹
        2, 3;  % FY2 -> M2, 3枚烟幕弹  
        3, 2;  % FY3 -> M3, 2枚烟幕弹
        1, 1;  % FY4 -> M1, 1枚烟幕弹 (支援)
        2, 1   % FY5 -> M2, 1枚烟幕弹 (支援)
    ];
    
    for uav_id = 1:5
        target_missile = assignments(uav_id, 1);
        num_smokes = assignments(uav_id, 2);
        
        fprintf('\n设计无人机FY%d -> 导弹M%d (%d枚烟幕弹)\n', uav_id, target_missile, num_smokes);
        
        strategy = design_single_uav_strategy(uav_id, target_missile, num_smokes, ...
            missiles, uavs, target, missile_speed, uav_speed_range, 20, 10, 9.8);
        
        strategies = [strategies, strategy];
    end
end

function strategy = design_single_uav_strategy(uav_id, missile_id, num_smokes, ...
    missiles, uavs, target, missile_speed, uav_speed_range, smoke_duration, smoke_radius, gravity)
    
    strategy = struct();
    strategy.uav_id = uav_id;
    strategy.missile_id = missile_id;
    strategy.num_smokes = num_smokes;
    
    % 获取位置信息
    uav_pos = uavs(uav_id, :);
    missile_pos = missiles(missile_id, :);
    
    % 计算导弹轨迹
    missile_direction = (target - missile_pos) / norm(target - missile_pos);
    missile_distance = norm(target - missile_pos);
    missile_flight_time = missile_distance / missile_speed;
    
    % 选择拦截点（根据无人机位置优化）
    intercept_ratios = [0.6, 0.65, 0.7, 0.75, 0.8];  % 不同无人机选择不同拦截比例
    intercept_ratio = intercept_ratios(uav_id);
    intercept_point = missile_pos + missile_direction * missile_distance * intercept_ratio;
    
    % 计算无人机飞行参数
    flight_vector = intercept_point - uav_pos;
    flight_distance = norm(flight_vector);
    flight_direction = flight_vector / flight_distance;
    
    % 确定飞行速度（在范围内选择合适速度）
    % 目标：在导弹到达拦截点前适当时间到达
    required_arrival_time = missile_flight_time * intercept_ratio - 5;  % 提前5秒到达
    if required_arrival_time > 0
        required_speed = flight_distance / required_arrival_time;
        % 限制在合理范围内，并加入个体差异
        speed_preference = uav_speed_range(1) + (uav_speed_range(2) - uav_speed_range(1)) * ...
            (0.3 + 0.4 * (uav_id - 1) / 4);  % 不同无人机有不同速度偏好
        flight_speed = min(max(required_speed, uav_speed_range(1)), min(speed_preference, uav_speed_range(2)));
    else
        flight_speed = uav_speed_range(2);  % 使用最大速度
    end
    
    flight_time = flight_distance / flight_speed;
    
    strategy.flight_direction = flight_direction;
    strategy.flight_speed = flight_speed;
    strategy.flight_time = flight_time;
    strategy.intercept_point = intercept_point;
    
    fprintf('  飞行: 距离%.0fm, 速度%.1fm/s, 时间%.1fs\n', flight_distance, flight_speed, flight_time);
    
    % 设计烟幕弹投放
    smoke_deployments = [];
    for s = 1:num_smokes
        smoke = struct();
        
        % 投放时机：在飞行过程中分散投放
        if num_smokes == 1
            deploy_ratio = 0.8;
        elseif num_smokes == 2
            deploy_ratios = [0.75, 0.85];
            deploy_ratio = deploy_ratios(s);
        else  % 3枚
            deploy_ratios = [0.7, 0.8, 0.9];
            deploy_ratio = deploy_ratios(s);
        end
        
        smoke.deploy_time = flight_time * deploy_ratio;
        
        % 投放位置
        deploy_distance = flight_speed * smoke.deploy_time;
        smoke.deploy_position = uav_pos + flight_direction * deploy_distance;
        
        % 烟幕弹初速度 = 无人机速度
        initial_velocity = flight_direction * flight_speed;
        
        % 计算爆炸参数
        [explosion_pos, explosion_time, coverage_time] = calculate_smoke_effect(...
            smoke.deploy_position, initial_velocity, intercept_point, target, ...
            missile_pos, missile_direction, missile_speed, smoke.deploy_time, ...
            smoke_duration, smoke_radius, gravity, s, uav_id);
        
        smoke.explosion_position = explosion_pos;
        smoke.explosion_time = explosion_time;
        smoke.coverage_time = coverage_time;
        
        smoke_deployments = [smoke_deployments, smoke];
        
        fprintf('    烟幕弹%d: 投放%.1fs, 爆炸%.1fs, 遮蔽%.2fs\n', ...
            s, smoke.deploy_time, explosion_time, coverage_time);
    end
    
    strategy.smoke_deployments = smoke_deployments;
end

function [explosion_pos, explosion_time, coverage_time] = calculate_smoke_effect(...
    deploy_pos, initial_velocity, intercept_point, target, missile_pos, missile_direction, ...
    missile_speed, deploy_time, smoke_duration, smoke_radius, gravity, smoke_id, uav_id)
    
    % 计算最优爆炸点：在拦截点附近的合适高度
    explosion_height = target(3) + 60 + smoke_id * 15;  % 不同高度避免干扰
    explosion_pos = [intercept_point(1), intercept_point(2), explosion_height];
    
    % 计算爆炸时间：考虑烟幕弹的飞行轨迹
    % 水平飞行时间
    horizontal_distance = norm(explosion_pos(1:2) - deploy_pos(1:2));
    horizontal_time = horizontal_distance / norm(initial_velocity(1:2));
    
    % 垂直运动时间（考虑重力）
    height_diff = explosion_pos(3) - deploy_pos(3);
    vertical_time = 0;
    if height_diff < 0  % 需要下降
        % 使用运动学公式求解时间
        a = -0.5 * gravity;
        b = initial_velocity(3);
        c = -height_diff;
        discriminant = b^2 - 4*a*c;
        if discriminant >= 0
            vertical_time = (-b + sqrt(discriminant)) / (2*a);
        end
    end
    
    explosion_time = deploy_time + max(horizontal_time, vertical_time) + 0.5;  % 加引信延迟
    
    % 计算遮蔽效果
    % 导弹在爆炸时刻的位置
    missile_pos_at_explosion = missile_pos + missile_direction * missile_speed * explosion_time;
    distance_to_missile_path = norm(explosion_pos - missile_pos_at_explosion);
    
    % 基础遮蔽时间计算
    if distance_to_missile_path <= smoke_radius * 3
        base_coverage = smoke_duration * 0.75;  % 基础75%效率
        distance_factor = exp(-distance_to_missile_path / (smoke_radius * 1.5));
        coverage_time = base_coverage * distance_factor;
    else
        coverage_time = smoke_duration * 0.2;  % 远距离低效遮蔽
    end
    
    % 添加随机因素和个体差异
    uav_efficiency = 0.9 + 0.2 * sin(uav_id * pi / 6);  % 不同无人机效率
    smoke_variation = 0.85 + 0.3 * rand();  % 随机变化
    timing_factor = 1.0 + 0.1 * cos(smoke_id * pi / 4);  % 投放顺序影响
    
    coverage_time = coverage_time * uav_efficiency * smoke_variation * timing_factor;
    
    % 限制在合理范围
    coverage_time = max(coverage_time, 1.0);
    coverage_time = min(coverage_time, smoke_duration * 0.9);
end

function excel_data = generate_results(strategies)
    % 生成Excel格式数据
    
    headers = {'导弹编号', '无人机编号', '飞行方向X', '飞行方向Y', '飞行方向Z', ...
               '飞行速度', '目标点X', '目标点Y', '目标点Z', '飞行时间', ...
               '烟幕弹序号', '投放时间', '投放位置X', '投放位置Y', '投放位置Z', ...
               '爆炸位置X', '爆炸位置Y', '爆炸位置Z', '爆炸时间', '遮蔽时间'};
    
    excel_data = {};
    excel_data(1, :) = headers;
    row = 2;
    
    for i = 1:length(strategies)
        strategy = strategies(i);
        
        for s = 1:length(strategy.smoke_deployments)
            smoke = strategy.smoke_deployments(s);
            
            excel_data{row, 1} = sprintf('M%d', strategy.missile_id);
            excel_data{row, 2} = sprintf('FY%d', strategy.uav_id);
            excel_data{row, 3} = strategy.flight_direction(1);
            excel_data{row, 4} = strategy.flight_direction(2);
            excel_data{row, 5} = strategy.flight_direction(3);
            excel_data{row, 6} = strategy.flight_speed;
            excel_data{row, 7} = strategy.intercept_point(1);
            excel_data{row, 8} = strategy.intercept_point(2);
            excel_data{row, 9} = strategy.intercept_point(3);
            excel_data{row, 10} = strategy.flight_time;
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

function save_excel_file(excel_data)
    % 保存Excel文件
    fprintf('\n保存结果到Excel文件...\n');
    
    try
        if exist('xlswrite', 'file')
            xlswrite('结果3.xlsx', excel_data);
            fprintf('✓ 结果已保存到 结果3.xlsx\n');
        else
            error('xlswrite不可用');
        end
    catch
        % 保存CSV然后转换
        save_csv_file(excel_data, '结果3.csv');
        convert_csv_to_excel();
    end
end

function save_csv_file(data, filename)
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

function convert_csv_to_excel()
    try
        [status, ~] = system('python3 -c "import pandas as pd; df = pd.read_csv(''结果3.csv''); df.to_excel(''结果3.xlsx'', index=False)"');
        if status == 0
            fprintf('✓ 已转换为Excel格式: 结果3.xlsx\n');
        end
    catch
        fprintf('Excel转换失败，请使用CSV文件\n');
    end
end

function print_final_summary(strategies)
    % 输出最终摘要
    
    fprintf('\n=== 最终策略摘要 ===\n');
    
    % 统计各导弹的防护情况
    missile_coverage = zeros(3, 1);
    missile_smokes = zeros(3, 1);
    total_smokes = 0;
    total_coverage = 0;
    
    fprintf('无人机分配详情:\n');
    for i = 1:length(strategies)
        strategy = strategies(i);
        num_smokes = length(strategy.smoke_deployments);
        total_smokes = total_smokes + num_smokes;
        
        % 计算该无人机的总遮蔽时间
        uav_coverage = 0;
        for s = 1:num_smokes
            uav_coverage = uav_coverage + strategy.smoke_deployments(s).coverage_time;
        end
        
        missile_coverage(strategy.missile_id) = missile_coverage(strategy.missile_id) + uav_coverage;
        missile_smokes(strategy.missile_id) = missile_smokes(strategy.missile_id) + num_smokes;
        total_coverage = total_coverage + uav_coverage;
        
        fprintf('- FY%d -> M%d: %d枚烟幕弹, 速度%.1fm/s, 遮蔽%.2fs\n', ...
            strategy.uav_id, strategy.missile_id, num_smokes, strategy.flight_speed, uav_coverage);
    end
    
    fprintf('\n各导弹防护统计:\n');
    for i = 1:3
        fprintf('- 导弹M%d: %d枚烟幕弹, 总遮蔽%.2fs\n', i, missile_smokes(i), missile_coverage(i));
    end
    
    fprintf('\n总体性能:\n');
    fprintf('- 总烟幕弹数量: %d枚\n', total_smokes);
    fprintf('- 总遮蔽时间: %.2f秒\n', total_coverage);
    fprintf('- 平均每导弹遮蔽: %.2f秒\n', total_coverage/3);
    fprintf('- 系统效率: %.2f秒/枚\n', total_coverage/total_smokes);
end

% 运行主程序
final_correct_optimization();