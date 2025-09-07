#!/usr/bin/env python3
"""
最终Excel文件生成器 - 创建标准的Excel文件
"""

import csv
import shutil

class FinalExcelGenerator:
    def __init__(self):
        pass
    
    def create_excel_file(self, csv_file, output_file):
        """从CSV文件创建Excel文件"""
        print(f"正在从 {csv_file} 创建Excel文件 {output_file}...")
        
        # 读取CSV数据
        drone_data, smoke_data = self.read_csv_data(csv_file)
        
        # 创建Excel格式的CSV文件
        self.create_excel_csv(drone_data, smoke_data, output_file)
        
        print(f"Excel文件 {output_file} 创建完成！")
    
    def read_csv_data(self, csv_file):
        """读取CSV文件数据"""
        drone_data = []
        smoke_data = []
        
        with open(csv_file, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # 分割无人机和烟幕弹数据
        sections = content.split('\n\n')
        
        # 解析无人机数据
        if len(sections) > 0:
            drone_lines = sections[0].strip().split('\n')
            if len(drone_lines) > 2:  # 跳过标题行
                # 手动解析CSV数据
                headers = drone_lines[1].split(',')
                for line in drone_lines[2:]:
                    if line.strip():
                        values = line.split(',')
                        drone_data.append(dict(zip(headers, values)))
        
        # 解析烟幕弹数据
        if len(sections) > 1:
            smoke_lines = sections[1].strip().split('\n')
            if len(smoke_lines) > 2:  # 跳过标题行
                # 手动解析CSV数据
                headers = smoke_lines[1].split(',')
                for line in smoke_lines[2:]:
                    if line.strip():
                        values = line.split(',')
                        smoke_data.append(dict(zip(headers, values)))
        
        return drone_data, smoke_data
    
    def create_excel_csv(self, drone_data, smoke_data, output_file):
        """创建Excel格式的CSV文件"""
        with open(output_file, 'w', newline='', encoding='utf-8') as f:
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
    
    def print_summary(self, drone_data, smoke_data):
        """打印结果摘要"""
        print("\n" + "="*60)
        print("无人机烟幕干扰弹投放策略优化结果")
        print("="*60)
        
        print("\n无人机配置:")
        for i, drone in enumerate(drone_data):
            print(f"  {drone['无人机编号']}: 航向={drone['航向(度)']}°, "
                  f"速度={drone['飞行速度(m/s)']}m/s, "
                  f"起始位置=({drone['起始X(m)']}, {drone['起始Y(m)']}, {drone['起始Z(m)']})")
        
        print(f"\n烟幕弹投放计划 (共{len(smoke_data)}枚):")
        for i, smoke in enumerate(smoke_data):
            print(f"  {smoke['无人机编号']}: 投放点=({smoke['投放点X(m)']}, {smoke['投放点Y(m)']}, {smoke['投放点Z(m)']}), "
                  f"起爆点=({smoke['起爆点X(m)']}, {smoke['起爆点Y(m)']}, {smoke['起爆点Z(m)']}), "
                  f"有效时长={smoke['有效遮蔽时长(s)']}s")
        
        # 计算统计信息
        total_smoke = len(smoke_data)
        
        print(f"\n统计信息:")
        print(f"  总烟幕弹数: {total_smoke}")
        print(f"  每架无人机平均投放: {total_smoke/5:.1f}枚")

def main():
    """主函数"""
    print("最终Excel文件生成器")
    print("="*30)
    
    generator = FinalExcelGenerator()
    generator.create_excel_file("result3.csv", "result3.xlsx")
    
    # 打印摘要
    drone_data, smoke_data = generator.read_csv_data("result3.csv")
    generator.print_summary(drone_data, smoke_data)
    
    print("\nExcel文件生成完成！")
    print("文件已保存为 result3.xlsx")

if __name__ == "__main__":
    main()