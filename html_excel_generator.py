#!/usr/bin/env python3
"""
HTML Excel文件生成器 - 创建可以在Excel中打开的HTML文件
"""

import csv
import html

class HTMLExcelGenerator:
    def __init__(self):
        pass
    
    def create_excel_file(self, csv_file, output_file):
        """从CSV文件创建HTML格式的Excel文件"""
        print(f"正在从 {csv_file} 创建Excel文件 {output_file}...")
        
        # 读取CSV数据
        drone_data, smoke_data = self.read_csv_data(csv_file)
        
        # 创建HTML格式的Excel文件
        self.create_html_excel(drone_data, smoke_data, output_file)
        
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
                reader = csv.DictReader(drone_lines[2:])
                drone_data = list(reader)
        
        # 解析烟幕弹数据
        if len(sections) > 1:
            smoke_lines = sections[1].strip().split('\n')
            if len(smoke_lines) > 2:  # 跳过标题行
                reader = csv.DictReader(smoke_lines[2:])
                smoke_data = list(reader)
        
        return drone_data, smoke_data
    
    def create_html_excel(self, drone_data, smoke_data, output_file):
        """创建HTML格式的Excel文件"""
        html_content = """<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>无人机烟幕干扰弹投放策略</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; }
        table { border-collapse: collapse; width: 100%; margin-bottom: 30px; }
        th, td { border: 1px solid #ddd; padding: 8px; text-align: center; }
        th { background-color: #f2f2f2; font-weight: bold; }
        .title { font-size: 18px; font-weight: bold; margin-bottom: 10px; }
        .section { margin-bottom: 30px; }
    </style>
</head>
<body>
    <h1>无人机烟幕干扰弹投放策略优化结果</h1>
"""
        
        # 添加无人机信息表
        html_content += self.create_drone_table(drone_data)
        
        # 添加烟幕弹信息表
        html_content += self.create_smoke_table(smoke_data)
        
        html_content += """
</body>
</html>"""
        
        # 写入文件
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write(html_content)
    
    def create_drone_table(self, drone_data):
        """创建无人机信息表"""
        if not drone_data:
            return ""
        
        html = """
    <div class="section">
        <div class="title">无人机信息</div>
        <table>
            <thead>
                <tr>
"""
        
        # 表头
        for key in drone_data[0].keys():
            html += f"                    <th>{key}</th>\n"
        
        html += """                </tr>
            </thead>
            <tbody>
"""
        
        # 数据行
        for drone in drone_data:
            html += "                <tr>\n"
            for key in drone_data[0].keys():
                html += f"                    <td>{str(drone[key])}</td>\n"
            html += "                </tr>\n"
        
        html += """            </tbody>
        </table>
    </div>
"""
        
        return html
    
    def create_smoke_table(self, smoke_data):
        """创建烟幕弹信息表"""
        if not smoke_data:
            return ""
        
        html = """
    <div class="section">
        <div class="title">烟幕弹信息</div>
        <table>
            <thead>
                <tr>
"""
        
        # 表头
        for key in smoke_data[0].keys():
            html += f"                    <th>{key}</th>\n"
        
        html += """                </tr>
            </thead>
            <tbody>
"""
        
        # 数据行
        for smoke in smoke_data:
            html += "                <tr>\n"
            for key in smoke_data[0].keys():
                html += f"                    <td>{str(smoke[key])}</td>\n"
            html += "                </tr>\n"
        
        html += """            </tbody>
        </table>
    </div>
"""
        
        return html

def main():
    """主函数"""
    print("HTML Excel文件生成器")
    print("="*30)
    
    generator = HTMLExcelGenerator()
    generator.create_excel_file("result3.csv", "result3.xlsx")
    
    print("\nExcel文件生成完成！")
    print("注意：生成的是HTML格式的Excel文件")
    print("可以在Excel中直接打开，或者用浏览器查看")

if __name__ == "__main__":
    main()