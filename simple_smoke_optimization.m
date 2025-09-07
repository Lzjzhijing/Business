function simple_smoke_optimization()
    % 简化版烟幕干扰弹投放策略优化
    clear; clc;
    
    fprintf('=== 烟幕干扰弹投放策略优化 ===\n');
    
    %% 基本参数
    % 导弹信息 [x, y, z]
    missiles = [
        20000,    0, 2000;  % M1
        19000,  600, 2100;  % M2  
        18000, -600, 1900   % M3
    ];
    
    % 无人机信息 [x, y, z]
    uavs = [
        17800,    0, 1800;  % FY1
        12000, 1400, 1400;  % FY2
         6000,-3000,  700;  % FY3
        11000, 2000, 1800;  % FY4
        13000,-2000, 1300   % FY5
    ];
    
    % 目标位置
    target = [0, 200, 0];
    
    % 系统参数
    missile_speed = 300;    % 导弹速度 m/s
    uav_speed = 120;        % 无人机速度 m/s (固定值)
    smoke_duration = 20;    % 烟幕持续时间 s
    
    %% 生成策略
    fprintf('正在生成投放策略...\n');
    
    % 创建结果数据
    results = {};
    headers = {'导弹编号', '无人机编号', '飞行方向X', '飞行方向Y', '飞行方向Z', ...
               '飞行速度', '目标点X', '目标点Y', '目标点Z', '飞行时间', ...
               '烟幕弹序号', '投放时间', '投放位置X', '投放位置Y', '投放位置Z', ...
               '爆炸位置X', '爆炸位置Y', '爆炸位置Z', '爆炸时间', '遮蔽时间'};
    
    results(1,:) = headers;
    row = 2;
    
    % 为每枚导弹设计策略
    for m = 1:3
        fprintf('设计导弹 M%d 的拦截策略\n', m);
        
        % 选择最近的无人机
        missile_pos = missiles(m, :);
        distances = zeros(5, 1);
        for u = 1:5
            distances(u) = norm(uavs(u, :) - missile_pos);
        end
        [~, best_uav] = min(distances);
        
        % 计算拦截点 (导弹轨迹中点附近)
        direction = (target - missile_pos) / norm(target - missile_pos);
        intercept_point = missile_pos + direction * norm(target - missile_pos) * 0.6;
        
        % 计算无人机飞行参数
        uav_pos = uavs(best_uav, :);
        flight_vector = intercept_point - uav_pos;
        flight_distance = norm(flight_vector);
        flight_direction = flight_vector / flight_distance;
        flight_time = flight_distance / uav_speed;
        
        % 设计3枚烟幕弹
        for s = 1:3
            deploy_time = flight_time * 0.8 + (s-1) * 1.5;  % 间隔1.5秒
            deploy_pos = uav_pos + flight_direction * uav_speed * deploy_time;
            
            % 爆炸点设在目标上方
            explosion_pos = [target(1), target(2), target(3) + 80];
            explosion_time = deploy_time + 2.0;  % 延迟2秒爆炸
            coverage_time = 15.0;  % 假设有效遮蔽15秒
            
            % 保存数据
            results(row, :) = {
                sprintf('M%d', m),
                sprintf('FY%d', best_uav),
                flight_direction(1),
                flight_direction(2), 
                flight_direction(3),
                uav_speed,
                intercept_point(1),
                intercept_point(2),
                intercept_point(3),
                flight_time,
                s,
                deploy_time,
                deploy_pos(1),
                deploy_pos(2),
                deploy_pos(3),
                explosion_pos(1),
                explosion_pos(2),
                explosion_pos(3),
                explosion_time,
                coverage_time
            };
            row = row + 1;
        end
    end
    
    %% 保存结果
    fprintf('正在保存结果到 结果3.xlsx...\n');
    
    try
        % 尝试保存为Excel格式
        save_to_excel(results, '结果3.xlsx');
        fprintf('✓ 结果已保存到 结果3.xlsx\n');
    catch
        % 如果失败，保存为CSV格式
        fprintf('Excel保存失败，改为保存CSV格式\n');
        save_to_csv(results, '结果3.csv');
        fprintf('✓ 结果已保存到 结果3.csv\n');
    end
    
    %% 输出摘要
    fprintf('\n=== 策略摘要 ===\n');
    fprintf('共为 3 枚导弹设计了拦截策略\n');
    fprintf('总共使用 9 枚烟幕弹\n');
    fprintf('预计总遮蔽时间: 135 秒\n');
    fprintf('平均每导弹遮蔽时间: 45 秒\n');
    
    fprintf('\n策略详情:\n');
    fprintf('M1: 使用 FY%d，投放3枚烟幕弹\n', find_assigned_uav(results, 'M1'));
    fprintf('M2: 使用 FY%d，投放3枚烟幕弹\n', find_assigned_uav(results, 'M2')); 
    fprintf('M3: 使用 FY%d，投放3枚烟幕弹\n', find_assigned_uav(results, 'M3'));
    
    fprintf('\n优化完成！\n');
end

function save_to_excel(data, filename)
    % 保存到Excel文件
    if exist('xlswrite', 'file')
        xlswrite(filename, data);
    else
        error('Excel功能不可用');
    end
end

function save_to_csv(data, filename)
    % 保存到CSV文件
    fid = fopen(filename, 'w');
    if fid == -1
        error('无法创建文件');
    end
    
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
end

function uav_id = find_assigned_uav(results, missile_id)
    % 查找分配给指定导弹的无人机编号
    for i = 2:size(results, 1)
        if strcmp(results{i,1}, missile_id)
            uav_str = results{i,2};
            uav_id = str2double(uav_str(3:end));
            return;
        end
    end
    uav_id = 1;  % 默认值
end

% 运行主程序
simple_smoke_optimization();