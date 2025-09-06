% 运行烟幕干扰弹投放策略优化
% Run Smoke Interference Optimization

clear; clc; close all;

fprintf('=== 烟幕干扰弹投放策略优化系统 ===\n');
fprintf('正在启动优化程序...\n\n');

try
    % 运行高级优化算法
    advanced_smoke_optimization();
    
    % 检查输出文件
    if exist('结果3.xlsx', 'file')
        fprintf('\n✓ 结果文件 结果3.xlsx 已成功生成\n');
    elseif exist('结果3.csv', 'file')
        fprintf('\n✓ 结果文件 结果3.csv 已成功生成\n');
    else
        fprintf('\n⚠ 未找到输出文件，请检查程序执行情况\n');
    end
    
    fprintf('\n程序执行完成！\n');
    
catch ME
    fprintf('\n❌ 程序执行出错:\n');
    fprintf('错误信息: %s\n', ME.message);
    fprintf('错误位置: %s (第 %d 行)\n', ME.stack(1).name, ME.stack(1).line);
    
    % 尝试运行基础版本
    fprintf('\n正在尝试运行基础版本...\n');
    try
        smoke_interference_optimization();
        fprintf('基础版本运行成功！\n');
    catch ME2
        fprintf('基础版本也运行失败: %s\n', ME2.message);
    end
end