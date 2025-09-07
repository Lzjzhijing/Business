#!/usr/bin/env python3
"""
高级无人机烟幕干扰弹投放策略求解器
考虑更精确的几何关系和优化策略
"""

import math
import csv

class AdvancedSmokeSolver:
    def __init__(self):
        """初始化问题参数"""
        # 导弹初始位置 (x, y, z)
        self.missiles = [
            [20000, 0, 2000],      # M1
            [19000, 600, 2100],    # M2
            [18000, -600, 1900]    # M3
        ]
        
        # 无人机初始位置 (x, y, z)
        self.drones = [
            [17800, 0, 1800],      # FY1
            [12000, 1400, 1400],   # FY2
            [6000, -3000, 700],    # FY3
            [11000, 2000, 1800],   # FY4
            [13000, -2000, 1300]   # FY5
        ]
        
        # 目标参数
        self.target_center = [0, 200, 0]  # 真目标中心
        self.fake_target = [0, 0, 0]      # 假目标位置
        
        # 物理参数
        self.missile_speed = 300          # 导弹速度 m/s
        self.drone_speed_min = 70         # 无人机最小速度 m/s
        self.drone_speed_max = 140        # 无人机最大速度 m/s
        self.smoke_sink_speed = 3         # 烟幕下沉速度 m/s
        self.smoke_effective_radius = 10  # 烟幕有效半径 m
        self.smoke_effective_duration = 20 # 烟幕有效持续时间 s
        self.smoke_interval_min = 1       # 烟幕弹投放最小间隔 s
        
        # 约束参数
        self.max_smoke_per_drone = 3      # 每架无人机最多投放烟幕弹数量
        self.num_drones = len(self.drones)
        self.num_missiles = len(self.missiles)
        
        # 时间步长
        self.time_step = 0.1
        self.max_simulation_time = 100
    
    def distance(self, pos1, pos2):
        """计算两点间距离"""
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(pos1, pos2)))
    
    def calculate_missile_trajectory(self, missile_idx, time):
        """计算导弹在指定时间的位置"""
        start_pos = self.missiles[missile_idx]
        direction = [self.fake_target[i] - start_pos[i] for i in range(3)]
        dist = math.sqrt(sum(x**2 for x in direction))
        direction = [x / dist for x in direction]
        
        return [start_pos[i] + direction[i] * self.missile_speed * time for i in range(3)]
    
    def calculate_drone_position(self, drone_idx, heading, speed, time):
        """计算无人机在指定时间的位置"""
        start_pos = self.drones[drone_idx]
        direction = [math.cos(math.radians(heading)), 
                    math.sin(math.radians(heading)), 0]
        
        return [start_pos[i] + direction[i] * speed * time for i in range(3)]
    
    def calculate_smoke_cloud_position(self, drop_pos, elapsed_time):
        """计算烟幕云团位置（考虑下沉）"""
        cloud_pos = drop_pos.copy()
        cloud_pos[2] = max(cloud_pos[2] - self.smoke_sink_speed * elapsed_time, 0)
        return cloud_pos
    
    def is_missile_covered(self, missile_pos, smoke_cloud_pos, elapsed_time):
        """检查导弹是否被烟幕覆盖"""
        if elapsed_time > self.smoke_effective_duration:
            return False
        
        distance = self.distance(missile_pos, smoke_cloud_pos)
        return distance <= self.smoke_effective_radius
    
    def calculate_coverage(self, solution):
        """计算整体覆盖率"""
        total_coverage = 0
        time_points = int(self.max_simulation_time / self.time_step)
        
        for t_idx in range(time_points):
            t = t_idx * self.time_step
            time_coverage = 0
            
            # 检查每枚导弹是否被覆盖
            for missile_idx in range(self.num_missiles):
                missile_pos = self.calculate_missile_trajectory(missile_idx, t)
                missile_covered = False
                
                # 检查所有烟幕弹
                for drone_idx in range(self.num_drones):
                    for smoke_idx in range(self.max_smoke_per_drone):
                        smoke_time = solution['smoke_times'][drone_idx][smoke_idx]
                        if smoke_time > 0 and t >= smoke_time:
                            # 计算烟幕弹投放位置
                            heading = solution['drone_headings'][drone_idx]
                            speed = solution['drone_speeds'][drone_idx]
                            drop_pos = self.calculate_drone_position(drone_idx, heading, speed, smoke_time)
                            
                            # 计算烟幕云团位置
                            elapsed_time = t - smoke_time
                            cloud_pos = self.calculate_smoke_cloud_position(drop_pos, elapsed_time)
                            
                            # 检查是否覆盖导弹
                            if self.is_missile_covered(missile_pos, cloud_pos, elapsed_time):
                                missile_covered = True
                                break
                    
                    if missile_covered:
                        break
                
                if missile_covered:
                    time_coverage += 1
            
            total_coverage += time_coverage
        
        return total_coverage / (time_points * self.num_missiles)
    
    def optimize_drone_mission(self, drone_idx):
        """优化单架无人机的任务"""
        best_coverage = 0
        best_heading = 0
        best_speed = self.drone_speed_min
        best_schedule = [0] * self.max_smoke_per_drone
        
        # 计算导弹到达时间
        arrival_times = []
        for missile_idx in range(self.num_missiles):
            dist = self.distance(self.missiles[missile_idx], self.fake_target)
            arrival_times.append(dist / self.missile_speed)
        
        # 尝试不同的航向和速度组合
        for heading in range(0, 360, 15):  # 每15度一个候选
            for speed in range(self.drone_speed_min, self.drone_speed_max + 1, 10):
                # 计算该配置下的最优烟幕弹投放时间
                smoke_schedule = self.optimize_smoke_timing(
                    drone_idx, heading, speed, arrival_times)
                
                # 创建临时解决方案
                temp_solution = {
                    'drone_headings': [0] * self.num_drones,
                    'drone_speeds': [0] * self.num_drones,
                    'smoke_times': [[0] * self.max_smoke_per_drone for _ in range(self.num_drones)]
                }
                temp_solution['drone_headings'][drone_idx] = heading
                temp_solution['drone_speeds'][drone_idx] = speed
                temp_solution['smoke_times'][drone_idx] = smoke_schedule
                
                # 计算覆盖率
                coverage = self.calculate_coverage(temp_solution)
                
                if coverage > best_coverage:
                    best_coverage = coverage
                    best_heading = heading
                    best_speed = speed
                    best_schedule = smoke_schedule
        
        print(f"无人机FY{drone_idx+1}: 航向={best_heading}°, 速度={best_speed}m/s, 覆盖率={best_coverage*100:.2f}%")
        return best_heading, best_speed, best_schedule
    
    def optimize_smoke_timing(self, drone_idx, heading, speed, arrival_times):
        """优化烟幕弹投放时间"""
        smoke_schedule = [0] * self.max_smoke_per_drone
        drone_pos = self.drones[drone_idx]
        
        # 计算无人机航线与导弹轨迹的交点
        intersection_times = []
        for missile_idx in range(self.num_missiles):
            t_intersect = self.find_intersection_time(
                drone_idx, heading, speed, missile_idx)
            
            if t_intersect > 0 and t_intersect <= self.max_simulation_time:
                intersection_times.append(t_intersect)
        
        if not intersection_times:
            return smoke_schedule
        
        # 按时间排序
        intersection_times.sort()
        
        # 分配烟幕弹
        smoke_count = 0
        last_smoke_time = 0
        
        for i, intersect_time in enumerate(intersection_times[:self.max_smoke_per_drone]):
            smoke_time = max(intersect_time - 3, 0)  # 提前3秒投放
            
            if smoke_time > last_smoke_time + self.smoke_interval_min:
                smoke_schedule[smoke_count] = smoke_time
                last_smoke_time = smoke_time
                smoke_count += 1
        
        return smoke_schedule
    
    def find_intersection_time(self, drone_idx, heading, speed, missile_idx):
        """计算无人机航线与导弹轨迹的交点时间"""
        drone_pos = self.drones[drone_idx]
        missile_pos = self.missiles[missile_idx]
        
        # 无人机方向向量
        drone_direction = [math.cos(math.radians(heading)), 
                          math.sin(math.radians(heading)), 0]
        
        # 导弹方向向量
        missile_direction = [self.fake_target[i] - missile_pos[i] for i in range(3)]
        missile_dist = math.sqrt(sum(x**2 for x in missile_direction))
        missile_direction = [x / missile_dist for x in missile_direction]
        
        # 求解交点时间
        # 无人机位置: drone_pos + drone_direction * drone_speed * t
        # 导弹位置: missile_pos + missile_direction * missile_speed * t
        
        # 建立线性方程组
        coeff_x = drone_direction[0] * speed - missile_direction[0] * self.missile_speed
        coeff_y = drone_direction[1] * speed - missile_direction[1] * self.missile_speed
        const_x = missile_pos[0] - drone_pos[0]
        const_y = missile_pos[1] - drone_pos[1]
        
        # 检查是否有解
        if abs(coeff_x) < 1e-10 and abs(coeff_y) < 1e-10:
            return -1
        
        # 使用x方向求解
        if abs(coeff_x) > 1e-10:
            t = const_x / coeff_x
        else:
            t = const_y / coeff_y
        
        return t if t > 0 else -1
    
    def solve(self):
        """主求解函数"""
        print("开始高级优化...")
        
        solution = {
            'drone_headings': [0] * self.num_drones,
            'drone_speeds': [0] * self.num_drones,
            'smoke_times': [[0] * self.max_smoke_per_drone for _ in range(self.num_drones)]
        }
        
        # 为每架无人机分配任务
        for drone_idx in range(self.num_drones):
            heading, speed, smoke_schedule = self.optimize_drone_mission(drone_idx)
            
            solution['drone_headings'][drone_idx] = heading
            solution['drone_speeds'][drone_idx] = speed
            solution['smoke_times'][drone_idx] = smoke_schedule
        
        # 计算最终覆盖率
        final_coverage = self.calculate_coverage(solution)
        print(f"\n最终整体覆盖率: {final_coverage*100:.2f}%")
        
        return solution
    
    def generate_results(self, solution):
        """生成结果文件"""
        # 准备无人机数据
        drone_data = []
        for i in range(self.num_drones):
            drone_data.append({
                '无人机编号': f'FY{i+1}',
                '航向(度)': solution['drone_headings'][i],
                '飞行速度(m/s)': solution['drone_speeds'][i],
                '起始X(m)': self.drones[i][0],
                '起始Y(m)': self.drones[i][1],
                '起始Z(m)': self.drones[i][2]
            })
        
        # 准备烟幕弹数据
        smoke_data = []
        for i in range(self.num_drones):
            for j in range(self.max_smoke_per_drone):
                smoke_time = solution['smoke_times'][i][j]
                if smoke_time > 0:
                    # 计算投放位置
                    heading = solution['drone_headings'][i]
                    speed = solution['drone_speeds'][i]
                    drop_pos = self.calculate_drone_position(i, heading, speed, smoke_time)
                    
                    # 起爆位置（假设瞬时起爆）
                    burst_pos = drop_pos.copy()
                    
                    smoke_data.append({
                        '无人机编号': f'FY{i+1}',
                        '投放点X(m)': round(drop_pos[0], 1),
                        '投放点Y(m)': round(drop_pos[1], 1),
                        '投放点Z(m)': round(drop_pos[2], 1),
                        '起爆点X(m)': round(burst_pos[0], 1),
                        '起爆点Y(m)': round(burst_pos[1], 1),
                        '起爆点Z(m)': round(burst_pos[2], 1),
                        '有效遮蔽时长(s)': self.smoke_effective_duration
                    })
        
        # 创建Excel格式的CSV文件
        self.create_excel_format(drone_data, smoke_data)
        
        print("结果已保存到 result3.csv")
        return drone_data, smoke_data
    
    def create_excel_format(self, drone_data, smoke_data):
        """创建Excel格式的CSV文件"""
        # 创建Excel格式的CSV文件
        with open('result3.csv', 'w', newline='', encoding='utf-8') as f:
            # 写入无人机信息
            f.write("无人机信息\n")
            if drone_data:
                writer = csv.DictWriter(f, fieldnames=drone_data[0].keys())
                writer.writeheader()
                writer.writerows(drone_data)
            
            f.write("\n\n烟幕弹信息\n")
            if smoke_data:
                writer = csv.DictWriter(f, fieldnames=smoke_data[0].keys())
                writer.writeheader()
                writer.writerows(smoke_data)
    
    def print_summary(self, solution, drone_data, smoke_data):
        """打印结果摘要"""
        print("\n" + "="*60)
        print("无人机烟幕干扰弹投放策略优化结果")
        print("="*60)
        
        print("\n无人机配置:")
        for i, drone in enumerate(drone_data):
            print(f"  {drone['无人机编号']}: 航向={drone['航向(度)']}°, "
                  f"速度={drone['飞行速度(m/s)']}m/s, "
                  f"起始位置=({drone['起始X(m)']:.0f}, {drone['起始Y(m)']:.0f}, {drone['起始Z(m)']:.0f})")
        
        print(f"\n烟幕弹投放计划 (共{len(smoke_data)}枚):")
        for i, smoke in enumerate(smoke_data):
            print(f"  {smoke['无人机编号']}: 投放点=({smoke['投放点X(m)']}, {smoke['投放点Y(m)']}, {smoke['投放点Z(m)']}), "
                  f"起爆点=({smoke['起爆点X(m)']}, {smoke['起爆点Y(m)']}, {smoke['起爆点Z(m)']}), "
                  f"有效时长={smoke['有效遮蔽时长(s)']}s")
        
        # 计算统计信息
        total_smoke = len(smoke_data)
        avg_drop_time = sum([smoke['投放点X(m)'] for smoke in smoke_data]) / total_smoke if total_smoke > 0 else 0
        
        print(f"\n统计信息:")
        print(f"  总烟幕弹数: {total_smoke}")
        print(f"  每架无人机平均投放: {total_smoke/self.num_drones:.1f}枚")
        
        # 计算覆盖率
        final_coverage = self.calculate_coverage(solution)
        print(f"  整体覆盖率: {final_coverage*100:.2f}%")

def main():
    """主函数"""
    print("高级无人机烟幕干扰弹投放策略优化求解器")
    print("="*50)
    
    # 创建求解器
    solver = AdvancedSmokeSolver()
    
    # 计算导弹到达时间
    arrival_times = []
    for missile_idx in range(solver.num_missiles):
        dist = solver.distance(solver.missiles[missile_idx], solver.fake_target)
        arrival_times.append(dist / solver.missile_speed)
    
    print(f"导弹到达时间: M1={arrival_times[0]:.1f}s, M2={arrival_times[1]:.1f}s, M3={arrival_times[2]:.1f}s")
    
    # 求解
    solution = solver.solve()
    
    # 生成结果
    drone_data, smoke_data = solver.generate_results(solution)
    
    # 打印摘要
    solver.print_summary(solution, drone_data, smoke_data)
    
    print("\n优化完成！结果已保存到 result3.csv")

if __name__ == "__main__":
    main()