#!/usr/bin/env python3
"""
Excel文件生成器 - 将CSV转换为Excel格式
使用Python内置功能创建Excel兼容的文件
"""

import csv
import json
import xml.etree.ElementTree as ET

class ExcelGenerator:
    def __init__(self):
        self.workbook_data = {}
    
    def create_excel_file(self, csv_file, output_file):
        """从CSV文件创建Excel文件"""
        print(f"正在从 {csv_file} 创建Excel文件 {output_file}...")
        
        # 读取CSV数据
        drone_data, smoke_data = self.read_csv_data(csv_file)
        
        # 创建Excel格式的XML
        self.create_excel_xml(drone_data, smoke_data, output_file)
        
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
    
    def create_excel_xml(self, drone_data, smoke_data, output_file):
        """创建Excel格式的XML文件"""
        # 创建根元素
        root = ET.Element("Workbook")
        root.set("xmlns", "urn:schemas-microsoft-com:office:spreadsheet")
        root.set("xmlns:o", "urn:schemas-microsoft-com:office:office")
        root.set("xmlns:x", "urn:schemas-microsoft-com:office:excel")
        root.set("xmlns:ss", "urn:schemas-microsoft-com:office:spreadsheet")
        root.set("xmlns:html", "http://www.w3.org/TR/REC-html40")
        
        # 创建样式
        styles = ET.SubElement(root, "Styles")
        style = ET.SubElement(styles, "Style")
        style.set("ss:ID", "Default")
        
        # 创建工作表
        worksheet = ET.SubElement(root, "Worksheet")
        worksheet.set("ss:Name", "无人机烟幕干扰弹投放策略")
        
        table = ET.SubElement(worksheet, "Table")
        
        # 添加无人机信息表
        self.add_drone_table(table, drone_data)
        
        # 添加空行
        empty_row = ET.SubElement(table, "Row")
        empty_cell = ET.SubElement(empty_row, "Cell")
        empty_cell.text = ""
        
        # 添加烟幕弹信息表
        self.add_smoke_table(table, smoke_data)
        
        # 写入文件
        tree = ET.ElementTree(root)
        tree.write(output_file, encoding='utf-8', xml_declaration=True)
    
    def add_drone_table(self, table, drone_data):
        """添加无人机信息表"""
        # 标题行
        title_row = ET.SubElement(table, "Row")
        title_cell = ET.SubElement(title_row, "Cell")
        title_cell.set("ss:StyleID", "Default")
        title_cell.text = "无人机信息"
        
        # 空行
        empty_row = ET.SubElement(table, "Row")
        empty_cell = ET.SubElement(empty_row, "Cell")
        empty_cell.text = ""
        
        if not drone_data:
            return
        
        # 表头
        header_row = ET.SubElement(table, "Row")
        for key in drone_data[0].keys():
            cell = ET.SubElement(header_row, "Cell")
            cell.set("ss:StyleID", "Default")
            cell.text = key
        
        # 数据行
        for drone in drone_data:
            row = ET.SubElement(table, "Row")
            for key in drone_data[0].keys():
                cell = ET.SubElement(row, "Cell")
                cell.set("ss:StyleID", "Default")
                cell.text = str(drone[key])
    
    def add_smoke_table(self, table, smoke_data):
        """添加烟幕弹信息表"""
        # 标题行
        title_row = ET.SubElement(table, "Row")
        title_cell = ET.SubElement(title_row, "Cell")
        title_cell.set("ss:StyleID", "Default")
        title_cell.text = "烟幕弹信息"
        
        # 空行
        empty_row = ET.SubElement(table, "Row")
        empty_cell = ET.SubElement(empty_row, "Cell")
        empty_cell.text = ""
        
        if not smoke_data:
            return
        
        # 表头
        header_row = ET.SubElement(table, "Row")
        for key in smoke_data[0].keys():
            cell = ET.SubElement(header_row, "Cell")
            cell.set("ss:StyleID", "Default")
            cell.text = key
        
        # 数据行
        for smoke in smoke_data:
            row = ET.SubElement(table, "Row")
            for key in smoke_data[0].keys():
                cell = ET.SubElement(row, "Cell")
                cell.set("ss:StyleID", "Default")
                cell.text = str(smoke[key])

def main():
    """主函数"""
    print("Excel文件生成器")
    print("="*30)
    
    generator = ExcelGenerator()
    generator.create_excel_file("result3.csv", "result3.xlsx")
    
    print("\nExcel文件生成完成！")
    print("注意：由于环境限制，生成的是XML格式的Excel文件")
    print("可以在Excel中打开，或者使用其他工具转换为标准Excel格式")

if __name__ == "__main__":
    main()