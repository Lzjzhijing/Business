#!/usr/bin/env python3
"""
无人机烟幕干扰弹投放策略优化求解器
使用Python实现，生成Excel结果文件
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import warnings
warnings.filterwarnings('ignore')

class SmokeInterferenceSolver:
    def __init__(self):
        """初始化问题参数"""
        # 导弹初始位置 (x, y, z)
        self.missiles = np.array([
            [20000, 0, 2000],      # M1
            [19000, 600, 2100],    # M2
            [18000, -600, 1900]    # M3
        ])
        
        # 无人机初始位置 (x, y, z)
        self.drones = np.array([
            [17800, 0, 1800],      # FY1
            [12000, 1400, 1400],   # FY2
            [6000, -3000, 700],    # FY3
            [11000, 2000, 1800],   # FY4
            [13000, -2000, 1300]   # FY5
        ])
        
        # 目标参数
        self.target_center = np.array([0, 200, 0])  # 真目标中心
        self.target_radius = 7                      # 真目标半径
        self.target_height = 10                     # 真目标高度
        self.fake_target = np.array([0, 0, 0])      # 假目标位置
        
        # 物理参数
        self.missile_speed = 300                    # 导弹速度 m/s
        self.drone_speed_min = 70                   # 无人机最小速度 m/s
        self.drone_speed_max = 140                  # 无人机最大速度 m/s
        self.smoke_sink_speed = 3                   # 烟幕下沉速度 m/s
        self.smoke_effective_radius = 10            # 烟幕有效半径 m
        self.smoke_effective_duration = 20          # 烟幕有效持续时间 s
        self.smoke_interval_min = 1                 # 烟幕弹投放最小间隔 s
        
        # 约束参数
        self.max_smoke_per_drone = 3                # 每架无人机最多投放烟幕弹数量
        self.num_drones = len(self.drones)
        self.num_missiles = len(self.missiles)
        
        # 时间参数
        self.time_step = 0.1                        # 时间步长 s
        self.max_simulation_time = 100              # 最大仿真时间 s
        
    def calculate_missile_trajectories(self):
        """计算导弹轨迹"""
        time_vec = np.arange(0, self.max_simulation_time + self.time_step, self.time_step)
        trajectories = np.zeros((len(time_vec), self.num_missiles, 3))
        
        for i in range(self.num_missiles):
            start_pos = self.missiles[i]
            # 导弹直指假目标
            direction = self.fake_target - start_pos
            direction = direction / np.linalg.norm(direction)
            
            for t_idx, t in enumerate(time_vec):
                pos = start_pos + direction * self.missile_speed * t
                trajectories[t_idx, i] = pos
                
        return time_vec, trajectories
    
    def find_intersection(self, drone_pos, drone_direction, drone_speed, 
                         missile_pos, missile_target, missile_speed):
        """计算无人机航线与导弹轨迹的交点"""
        missile_direction = missile_target - missile_pos
        missile_direction = missile_direction / np.linalg.norm(missile_direction)
        
        # 建立方程组求解
        # 无人机位置: drone_pos + drone_direction * drone_speed * t
        # 导弹位置: missile_pos + missile_direction * missile_speed * t
        
        A = np.array([drone_direction * drone_speed, -missile_direction * missile_speed]).T
        b = missile_pos - drone_pos
        
        try:
            t_solution = np.linalg.solve(A, b)
            t_intersect = t_solution[0]
            
            if t_intersect > 0:
                pos_intersect = drone_pos + drone_direction * drone_speed * t_intersect
                return t_intersect, pos_intersect
            else:
                return -1, np.array([0, 0, 0])
        except np.linalg.LinAlgError:
            return -1, np.array([0, 0, 0])
    
    def calculate_coverage_for_drone(self, drone_idx, heading, speed, smoke_schedule, 
                                   time_vec, trajectories):
        """计算单架无人机的烟幕覆盖率"""
        coverage = 0
        drone_pos = self.drones[drone_idx]
        direction = np.array([np.cos(np.radians(heading)), np.sin(np.radians(heading)), 0])
        
        for t_idx, t in enumerate(time_vec):
            # 检查是否有烟幕提供遮蔽
            smoke_coverage = 0
            
            for smoke_time in smoke_schedule:
                if smoke_time > 0 and t >= smoke_time:
                    # 计算烟幕弹投放位置
                    drop_pos = drone_pos + direction * speed * smoke_time
                    
                    # 计算烟幕云团位置（考虑下沉）
                    elapsed_time = t - smoke_time
                    cloud_pos = drop_pos.copy()
                    cloud_pos[2] = cloud_pos[2] - self.smoke_sink_speed * elapsed_time
                    cloud_pos[2] = max(cloud_pos[2], 0)
                    
                    # 检查是否在有效时间内
                    if elapsed_time <= self.smoke_effective_duration:
                        # 检查是否覆盖任何导弹
                        for missile_idx in range(self.num_missiles):
                            missile_pos = trajectories[t_idx, missile_idx]
                            distance = np.linalg.norm(missile_pos - cloud_pos)
                            if distance <= self.smoke_effective_radius:
                                smoke_coverage = 1
                                break
                    
                    if smoke_coverage > 0:
                        break
            
            coverage += smoke_coverage
        
        # 归一化覆盖率
        return coverage / len(time_vec)
    
    def optimize_drone_mission(self, drone_idx, time_vec, trajectories, missile_arrival_times):
        """优化单架无人机的任务"""
        drone_pos = self.drones[drone_idx]
        best_coverage = 0
        best_heading = 0
        best_speed = self.drone_speed_min
        best_schedule = np.zeros(self.max_smoke_per_drone)
        
        # 尝试不同的航向和速度组合
        heading_candidates = np.arange(0, 360, 30)  # 每30度一个候选
        speed_candidates = np.arange(self.drone_speed_min, self.drone_speed_max + 1, 20)
        
        for heading in heading_candidates:
            for speed in speed_candidates:
                # 计算该配置下的最优烟幕弹投放时间
                smoke_schedule, coverage = self.optimize_smoke_timing(
                    drone_idx, heading, speed, time_vec, trajectories, missile_arrival_times)
                
                if coverage > best_coverage:
                    best_coverage = coverage
                    best_heading = heading
                    best_speed = speed
                    best_schedule = smoke_schedule
        
        print(f"无人机FY{drone_idx+1}: 航向={best_heading:.0f}°, 速度={best_speed:.0f}m/s, 覆盖率={best_coverage*100:.2f}%")
        return best_heading, best_speed, best_schedule
    
    def optimize_smoke_timing(self, drone_idx, heading, speed, time_vec, trajectories, missile_arrival_times):
        """优化烟幕弹投放时间"""
        smoke_schedule = np.zeros(self.max_smoke_per_drone)
        drone_pos = self.drones[drone_idx]
        direction = np.array([np.cos(np.radians(heading)), np.sin(np.radians(heading)), 0])
        
        # 计算导弹轨迹与无人机航线的交点
        intersection_times = np.zeros(self.num_missiles)
        intersection_positions = np.zeros((self.num_missiles, 3))
        
        for missile_idx in range(self.num_missiles):
            t_intersect, pos_intersect = self.find_intersection(
                drone_pos, direction, speed,
                self.missiles[missile_idx], self.fake_target, self.missile_speed)
            
            if t_intersect > 0 and t_intersect <= self.max_simulation_time:
                intersection_times[missile_idx] = t_intersect
                intersection_positions[missile_idx] = pos_intersect
        
        # 选择最佳的烟幕弹投放时间
        valid_intersections = np.where(intersection_times > 0)[0]
        if len(valid_intersections) == 0:
            return smoke_schedule, 0
        
        # 按时间排序
        sorted_indices = np.argsort(intersection_times[valid_intersections])
        sorted_times = intersection_times[valid_intersections][sorted_indices]
        
        # 分配烟幕弹
        smoke_count = 0
        last_smoke_time = 0
        
        for i in range(min(len(sorted_times), self.max_smoke_per_drone)):
            smoke_time = sorted_times[i] - 2  # 提前2秒投放
            
            if smoke_time > last_smoke_time + self.smoke_interval_min:
                smoke_schedule[smoke_count] = smoke_time
                last_smoke_time = smoke_time
                smoke_count += 1
        
        # 计算覆盖率
        coverage = self.calculate_coverage_for_drone(drone_idx, heading, speed, 
                                                   smoke_schedule, time_vec, trajectories)
        return smoke_schedule, coverage
    
    def solve(self):
        """主求解函数"""
        print("开始计算导弹轨迹...")
        time_vec, trajectories = self.calculate_missile_trajectories()
        
        # 计算导弹到达假目标的时间
        missile_arrival_times = np.zeros(self.num_missiles)
        for i in range(self.num_missiles):
            distance = np.linalg.norm(self.missiles[i] - self.fake_target)
            missile_arrival_times[i] = distance / self.missile_speed
        
        print(f"导弹到达时间: M1={missile_arrival_times[0]:.1f}s, M2={missile_arrival_times[1]:.1f}s, M3={missile_arrival_times[2]:.1f}s")
        
        # 为每架无人机分配任务
        solution = {
            'drone_headings': np.zeros(self.num_drones),
            'drone_speeds': np.zeros(self.num_drones),
            'smoke_times': np.zeros((self.num_drones, self.max_smoke_per_drone))
        }
        
        print("开始优化无人机任务...")
        for drone_idx in range(self.num_drones):
            heading, speed, smoke_schedule = self.optimize_drone_mission(
                drone_idx, time_vec, trajectories, missile_arrival_times)
            
            solution['drone_headings'][drone_idx] = heading
            solution['drone_speeds'][drone_idx] = speed
            solution['smoke_times'][drone_idx] = smoke_schedule
        
        return solution, time_vec, trajectories
    
    def generate_results(self, solution, time_vec, trajectories):
        """生成Excel结果文件"""
        # 准备无人机数据
        drone_data = []
        for i in range(self.num_drones):
            drone_data.append({
                '无人机编号': f'FY{i+1}',
                '航向(度)': solution['drone_headings'][i],
                '飞行速度(m/s)': solution['drone_speeds'][i],
                '起始X(m)': self.drones[i, 0],
                '起始Y(m)': self.drones[i, 1],
                '起始Z(m)': self.drones[i, 2]
            })
        
        # 准备烟幕弹数据
        smoke_data = []
        for i in range(self.num_drones):
            for j in range(self.max_smoke_per_drone):
                smoke_time = solution['smoke_times'][i, j]
                if smoke_time > 0:
                    # 计算投放位置
                    heading = solution['drone_headings'][i]
                    speed = solution['drone_speeds'][i]
                    direction = np.array([np.cos(np.radians(heading)), np.sin(np.radians(heading)), 0])
                    drop_pos = self.drones[i] + direction * speed * smoke_time
                    
                    # 起爆位置（假设瞬时起爆）
                    burst_pos = drop_pos.copy()
                    
                    smoke_data.append({
                        '无人机编号': f'FY{i+1}',
                        '投放点X(m)': drop_pos[0],
                        '投放点Y(m)': drop_pos[1],
                        '投放点Z(m)': drop_pos[2],
                        '起爆点X(m)': burst_pos[0],
                        '起爆点Y(m)': burst_pos[1],
                        '起爆点Z(m)': burst_pos[2],
                        '有效遮蔽时长(s)': self.smoke_effective_duration
                    })
        
        # 创建Excel文件
        with pd.ExcelWriter('result3.xlsx', engine='openpyxl') as writer:
            # 无人机工作表
            drone_df = pd.DataFrame(drone_data)
            drone_df.to_excel(writer, sheet_name='无人机信息', index=False)
            
            # 烟幕弹工作表
            smoke_df = pd.DataFrame(smoke_data)
            smoke_df.to_excel(writer, sheet_name='烟幕弹信息', index=False)
        
        print("结果已保存到 result3.xlsx")
        return drone_data, smoke_data
    
    def visualize_solution(self, solution, time_vec, trajectories):
        """可视化解决方案"""
        fig = plt.figure(figsize=(15, 10))
        
        # 3D视图
        ax1 = fig.add_subplot(221, projection='3d')
        
        # 绘制导弹轨迹
        colors = ['r', 'g', 'b']
        for i in range(self.num_missiles):
            trajectory = trajectories[:, i, :]
            ax1.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], 
                    color=colors[i], linewidth=2, label=f'M{i+1}轨迹')
            ax1.scatter(self.missiles[i, 0], self.missiles[i, 1], self.missiles[i, 2], 
                       color=colors[i], s=100, marker='o')
        
        # 绘制目标
        ax1.scatter(self.target_center[0], self.target_center[1], self.target_center[2], 
                   color='black', s=200, marker='o', label='真目标')
        ax1.scatter(self.fake_target[0], self.fake_target[1], self.fake_target[2], 
                   color='yellow', s=200, marker='s', label='假目标')
        
        # 绘制无人机轨迹和烟幕弹投放
        for i in range(self.num_drones):
            drone_pos = self.drones[i]
            heading = solution['drone_headings'][i]
            speed = solution['drone_speeds'][i]
            direction = np.array([np.cos(np.radians(heading)), np.sin(np.radians(heading)), 0])
            
            # 无人机轨迹
            t_end = np.max(solution['smoke_times'][i, :])
            if t_end > 0:
                t_vec = np.arange(0, t_end + 0.1, 0.1)
                drone_trajectory = drone_pos + direction * speed * t_vec[:, np.newaxis]
                ax1.plot(drone_trajectory[:, 0], drone_trajectory[:, 1], drone_trajectory[:, 2], 
                        'k--', linewidth=1, alpha=0.7)
            
            # 烟幕弹投放点
            for j in range(self.max_smoke_per_drone):
                smoke_time = solution['smoke_times'][i, j]
                if smoke_time > 0:
                    drop_pos = drone_pos + direction * speed * smoke_time
                    ax1.scatter(drop_pos[0], drop_pos[1], drop_pos[2], 
                               color='red', s=50, marker='o')
        
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_zlabel('Z (m)')
        ax1.set_title('3D 轨迹视图')
        ax1.legend()
        ax1.grid(True)
        
        # 俯视图
        ax2 = fig.add_subplot(222)
        
        # 绘制导弹轨迹（俯视）
        for i in range(self.num_missiles):
            trajectory = trajectories[:, i, :]
            ax2.plot(trajectory[:, 0], trajectory[:, 1], color=colors[i], linewidth=2, label=f'M{i+1}轨迹')
            ax2.scatter(self.missiles[i, 0], self.missiles[i, 1], color=colors[i], s=100, marker='o')
        
        # 绘制目标（俯视）
        ax2.scatter(self.target_center[0], self.target_center[1], color='black', s=200, marker='o', label='真目标')
        ax2.scatter(self.fake_target[0], self.fake_target[1], color='yellow', s=200, marker='s', label='假目标')
        
        # 绘制无人机和烟幕弹投放（俯视）
        for i in range(self.num_drones):
            ax2.scatter(self.drones[i, 0], self.drones[i, 1], color='blue', s=100, marker='^', label=f'FY{i+1}' if i == 0 else "")
            
            for j in range(self.max_smoke_per_drone):
                smoke_time = solution['smoke_times'][i, j]
                if smoke_time > 0:
                    heading = solution['drone_headings'][i]
                    speed = solution['drone_speeds'][i]
                    direction = np.array([np.cos(np.radians(heading)), np.sin(np.radians(heading)), 0])
                    drop_pos = self.drones[i] + direction * speed * smoke_time
                    ax2.scatter(drop_pos[0], drop_pos[1], color='red', s=50, marker='o')
        
        ax2.set_xlabel('X (m)')
        ax2.set_ylabel('Y (m)')
        ax2.set_title('俯视图')
        ax2.legend()
        ax2.grid(True)
        ax2.axis('equal')
        
        # 时间轴视图
        ax3 = fig.add_subplot(223)
        
        # 绘制导弹到达时间
        for i in range(self.num_missiles):
            arrival_time = np.linalg.norm(self.missiles[i] - self.fake_target) / self.missile_speed
            ax3.plot([0, arrival_time], [i, i], color=colors[i], linewidth=3, label=f'M{i+1}')
            ax3.text(arrival_time, i, f'M{i+1}', fontsize=10)
        
        # 绘制烟幕弹投放时间
        for i in range(self.num_drones):
            for j in range(self.max_smoke_per_drone):
                smoke_time = solution['smoke_times'][i, j]
                if smoke_time > 0:
                    ax3.scatter(smoke_time, i + 3, color='red', s=50, marker='o')
                    ax3.text(smoke_time, i + 3, f'FY{i+1}', fontsize=8)
        
        ax3.set_xlabel('时间 (s)')
        ax3.set_ylabel('导弹/无人机')
        ax3.set_title('时间轴视图')
        ax3.legend()
        ax3.grid(True)
        
        # 性能统计
        ax4 = fig.add_subplot(224)
        
        # 计算覆盖率统计
        total_coverage = 0
        for missile_idx in range(self.num_missiles):
            coverage = self.calculate_coverage_for_drone(0, solution['drone_headings'][0], 
                                                       solution['drone_speeds'][0], 
                                                       solution['smoke_times'][0, :], 
                                                       time_vec, trajectories)
            total_coverage += coverage
        
        avg_coverage = total_coverage / self.num_missiles
        total_smoke = np.sum(solution['smoke_times'] > 0)
        avg_drop_time = np.mean(solution['smoke_times'][solution['smoke_times'] > 0])
        
        # 显示统计信息
        ax4.text(0.1, 0.8, f'平均覆盖率: {avg_coverage*100:.2f}%', fontsize=12, transform=ax4.transAxes)
        ax4.text(0.1, 0.6, f'总烟幕弹数: {total_smoke}', fontsize=12, transform=ax4.transAxes)
        ax4.text(0.1, 0.4, f'平均投放时间: {avg_drop_time:.1f}s', fontsize=12, transform=ax4.transAxes)
        
        ax4.set_xlim(0, 1)
        ax4.set_ylim(0, 1)
        ax4.axis('off')
        ax4.set_title('性能统计')
        
        plt.tight_layout()
        plt.savefig('smoke_interference_result.png', dpi=300, bbox_inches='tight')
        plt.show()

def main():
    """主函数"""
    print("无人机烟幕干扰弹投放策略优化求解器")
    print("=" * 50)
    
    # 创建求解器
    solver = SmokeInterferenceSolver()
    
    # 求解
    solution, time_vec, trajectories = solver.solve()
    
    # 生成结果
    drone_data, smoke_data = solver.generate_results(solution, time_vec, trajectories)
    
    # 可视化
    solver.visualize_solution(solution, time_vec, trajectories)
    
    print("\n优化完成！")
    print(f"共投放 {len(smoke_data)} 枚烟幕弹")
    print("结果已保存到 result3.xlsx")

if __name__ == "__main__":
    main()