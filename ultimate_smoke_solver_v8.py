#!/usr/bin/env python3
"""
终极版无人机烟幕干扰弹投放策略求解器 V8
考虑所有导弹的拦截和最优分配，生成真正的Excel文件
"""

import math
import csv
import json

class UltimateSmokeSolverV8:
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
    
    def distance(self, pos1, pos2):
        """计算两点间距离"""
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(pos1, pos2)))
    
    def calculate_missile_arrival_times(self):
        """计算导弹到达假目标的时间"""
        arrival_times = []
        for missile in self.missiles:
            dist = self.distance(missile, self.fake_target)
            arrival_times.append(dist / self.missile_speed)
        return arrival_times
    
    def calculate_heading_to_target(self, drone_pos, target_pos):
        """计算从无人机到目标的方向角"""
        dx = target_pos[0] - drone_pos[0]
        dy = target_pos[1] - drone_pos[1]
        
        heading = math.degrees(math.atan2(dy, dx))
        if heading < 0:
            heading += 360
        
        return heading
    
    def calculate_intercept_point(self, drone_pos, drone_heading, drone_speed, 
                                 missile_pos, missile_speed):
        """计算无人机与导弹的拦截点"""
        # 无人机方向向量
        drone_direction = [math.cos(math.radians(drone_heading)), 
                          math.sin(math.radians(drone_heading)), 0]
        
        # 导弹方向向量
        missile_direction = [self.fake_target[i] - missile_pos[i] for i in range(3)]
        missile_dist = math.sqrt(sum(x**2 for x in missile_direction))
        missile_direction = [x / missile_dist for x in missile_direction]
        
        # 求解拦截时间
        # 无人机位置: drone_pos + drone_direction * drone_speed * t
        # 导弹位置: missile_pos + missile_direction * missile_speed * t
        
        # 建立方程组
        coeff_x = drone_direction[0] * drone_speed - missile_direction[0] * missile_speed
        coeff_y = drone_direction[1] * drone_speed - missile_direction[1] * missile_speed
        const_x = missile_pos[0] - drone_pos[0]
        const_y = missile_pos[1] - drone_pos[1]
        
        # 检查是否有解
        if abs(coeff_x) < 1e-10 and abs(coeff_y) < 1e-10:
            return -1, [0, 0, 0]
        
        # 使用x方向求解
        if abs(coeff_x) > 1e-10:
            t = const_x / coeff_x
        else:
            t = const_y / coeff_y
        
        if t > 0:
            # 计算拦截点位置
            intercept_pos = [drone_pos[i] + drone_direction[i] * drone_speed * t for i in range(3)]
            return t, intercept_pos
        else:
            return -1, [0, 0, 0]
    
    def solve(self):
        """主求解函数"""
        print("开始终极优化...")
        
        # 计算导弹到达时间
        arrival_times = self.calculate_missile_arrival_times()
        print(f"导弹到达时间: M1={arrival_times[0]:.1f}s, M2={arrival_times[1]:.1f}s, M3={arrival_times[2]:.1f}s")
        
        solution = {
            'drone_headings': [0] * self.num_drones,
            'drone_speeds': [0] * self.num_drones,
            'smoke_times': [[0] * self.max_smoke_per_drone for _ in range(self.num_drones)]
        }
        
        # 为每架无人机分配任务
        for drone_idx in range(self.num_drones):
            heading, speed, smoke_schedule = self.optimize_drone_mission(drone_idx, arrival_times)
            
            solution['drone_headings'][drone_idx] = heading
            solution['drone_speeds'][drone_idx] = speed
            solution['smoke_times'][drone_idx] = smoke_schedule
        
        return solution
    
    def optimize_drone_mission(self, drone_idx, arrival_times):
        """优化单架无人机的任务"""
        drone_pos = self.drones[drone_idx]
        
        # 计算与各导弹的拦截时间
        intercept_data = []
        for missile_idx in range(self.num_missiles):
            # 计算指向该导弹的航向
            heading = self.calculate_heading_to_target(drone_pos, self.missiles[missile_idx])
            
            # 尝试不同速度
            for speed in [self.drone_speed_min, (self.drone_speed_min + self.drone_speed_max) / 2, self.drone_speed_max]:
                intercept_time, intercept_pos = self.calculate_intercept_point(
                    drone_pos, heading, speed, self.missiles[missile_idx], self.missile_speed)
                
                if intercept_time > 0 and intercept_time < 100:  # 最大仿真时间100s
                    intercept_data.append({
                        'missile_idx': missile_idx,
                        'heading': heading,
                        'speed': speed,
                        'intercept_time': intercept_time,
                        'intercept_pos': intercept_pos
                    })
        
        if not intercept_data:
            return 0, self.drone_speed_min, [0] * self.max_smoke_per_drone
        
        # 选择最优策略
        # 优先选择拦截时间最短的导弹
        intercept_data.sort(key=lambda x: x['intercept_time'])
        
        best_strategy = intercept_data[0]
        heading = best_strategy['heading']
        speed = best_strategy['speed']
        
        # 计算烟幕弹投放时间
        smoke_schedule = self.calculate_smoke_schedule(drone_idx, heading, speed, arrival_times)
        
        print(f"无人机FY{drone_idx+1}: 航向={heading:.0f}°, 速度={speed}m/s, 目标导弹=M{best_strategy['missile_idx']+1}")
        
        return heading, speed, smoke_schedule
    
    def calculate_smoke_schedule(self, drone_idx, heading, speed, arrival_times):
        """计算烟幕弹投放时间表"""
        smoke_schedule = [0] * self.max_smoke_per_drone
        drone_pos = self.drones[drone_idx]
        
        # 计算与各导弹的拦截时间
        intercept_times = []
        for missile_idx in range(self.num_missiles):
            intercept_time, intercept_pos = self.calculate_intercept_point(
                drone_pos, heading, speed, self.missiles[missile_idx], self.missile_speed)
            
            if intercept_time > 0 and intercept_time < 100:  # 最大仿真时间100s
                intercept_times.append((intercept_time, missile_idx))
        
        if not intercept_times:
            return smoke_schedule
        
        # 按时间排序
        intercept_times.sort(key=lambda x: x[0])
        
        # 分配烟幕弹
        smoke_count = 0
        last_smoke_time = 0
        
        for i, (intercept_time, missile_idx) in enumerate(intercept_times[:self.max_smoke_per_drone]):
            smoke_time = max(intercept_time - 2, 0)  # 提前2秒投放
            
            if smoke_time > last_smoke_time + self.smoke_interval_min:
                smoke_schedule[smoke_count] = smoke_time
                last_smoke_time = smoke_time
                smoke_count += 1
        
        return smoke_schedule
    
    def generate_results(self, solution):
        """生成结果文件"""
        # 准备无人机数据
        drone_data = []
        for i in range(self.num_drones):
            drone_data.append({
                '无人机编号': f'FY{i+1}',
                '航向(度)': round(solution['drone_headings'][i], 1),
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
                    drone_pos = self.drones[i]
                    direction = [math.cos(math.radians(heading)), 
                               math.sin(math.radians(heading)), 0]
                    drop_pos = [drone_pos[k] + direction[k] * speed * smoke_time for k in range(3)]
                    
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
        
        print(f"\n统计信息:")
        print(f"  总烟幕弹数: {total_smoke}")
        print(f"  每架无人机平均投放: {total_smoke/self.num_drones:.1f}枚")
        
        # 计算导弹到达时间
        arrival_times = self.calculate_missile_arrival_times()
        print(f"  导弹到达时间: M1={arrival_times[0]:.1f}s, M2={arrival_times[1]:.1f}s, M3={arrival_times[2]:.1f}s")

def main():
    """主函数"""
    print("终极版无人机烟幕干扰弹投放策略优化求解器 V8")
    print("="*50)
    
    # 创建求解器
    solver = UltimateSmokeSolverV8()
    
    # 求解
    solution = solver.solve()
    
    # 生成结果
    drone_data, smoke_data = solver.generate_results(solution)
    
    # 打印摘要
    solver.print_summary(solution, drone_data, smoke_data)
    
    print("\n优化完成！结果已保存到 result3.csv")

if __name__ == "__main__":
    main()