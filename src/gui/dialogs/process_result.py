#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
处理结果展示对话框
"""

import os
import sys
import shutil
from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QLabel, 
                           QPushButton, QTableWidget, QTableWidgetItem,
                           QHeaderView, QMessageBox, QFrame, QWidget)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor, QFont


class ProcessResultDialog(QDialog):
    """处理结果展示对话框"""
    def __init__(self, results, output_dir, parent=None):
        super().__init__(parent)
        self.results = results
        self.output_dir = output_dir
        self.init_ui()
        
    def init_ui(self):
        self.setWindowTitle("数据处理结果")
        self.setMinimumSize(600, 400)
        
        layout = QVBoxLayout(self)
        
        # 标题
        title_label = QLabel("组帧同步处理结果")
        title_label.setFont(QFont('Arial', 14, QFont.Bold))
        title_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(title_label)
        
        # 结果表格
        self.result_table = QTableWidget(len(self.results), 4)
        self.result_table.setHorizontalHeaderLabels(["目录名", "状态", "帧数", "操作"])
        self.result_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.result_table.setSelectionMode(QTableWidget.SingleSelection)
        self.result_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        
        for i, result in enumerate(self.results):
            dir_name = result.get('dir_name', '')
            success = result.get('success', False)
            frame_count = result.get('frame_count', 0)
            
            # 目录名
            self.result_table.setItem(i, 0, QTableWidgetItem(dir_name))
            
            # 状态
            status_item = QTableWidgetItem("成功" if success else "失败")
            status_item.setForeground(QColor("green") if success else QColor("red"))
            self.result_table.setItem(i, 1, status_item)
            
            # 帧数
            self.result_table.setItem(i, 2, QTableWidgetItem(str(frame_count)))
            
            # 操作按钮单元格
            cell_widget = QWidget()
            cell_layout = QHBoxLayout(cell_widget)
            cell_layout.setContentsMargins(2, 2, 2, 2)
            
            # 查看按钮
            view_button = QPushButton("查看")
            view_button.setEnabled(success)
            view_button.clicked.connect(lambda checked, row=i: self.view_result(row))
            cell_layout.addWidget(view_button)
            
            # 删除按钮
            delete_button = QPushButton("删除")
            delete_button.setEnabled(success)
            delete_button.clicked.connect(lambda checked, row=i: self.delete_result(row))
            cell_layout.addWidget(delete_button)
            
            self.result_table.setCellWidget(i, 3, cell_widget)
        
        layout.addWidget(self.result_table)
        
        # 摘要信息
        summary_frame = QFrame()
        summary_frame.setFrameShape(QFrame.StyledPanel)
        summary_layout = QHBoxLayout(summary_frame)
        
        successful = sum(1 for r in self.results if r.get('success', False))
        total_frames = sum(r.get('frame_count', 0) for r in self.results)
        
        summary_layout.addWidget(QLabel(f"成功处理: {successful}/{len(self.results)}"))
        summary_layout.addWidget(QLabel(f"总帧数: {total_frames}"))
        summary_layout.addWidget(QLabel(f"保存目录: {self.output_dir}"))
        
        layout.addWidget(summary_frame)
        
        # 按钮区域
        button_layout = QHBoxLayout()
        
        open_dir_button = QPushButton("打开输出目录")
        open_dir_button.clicked.connect(self.open_output_dir)
        button_layout.addWidget(open_dir_button)
        
        close_button = QPushButton("关闭")
        close_button.clicked.connect(self.accept)
        button_layout.addWidget(close_button)
        
        layout.addLayout(button_layout)
    
    def view_result(self, row):
        """查看处理结果"""
        if row < 0 or row >= len(self.results):
            return
            
        result = self.results[row]
        if not result.get('success', False):
            return
            
        dir_name = result.get('dir_name', '')
        final_output_dir = os.path.join(self.output_dir, "final_output", dir_name)
        
        if not os.path.exists(final_output_dir):
            QMessageBox.warning(self, "错误", f"找不到输出目录: {final_output_dir}")
            return
            
        # 使用系统文件管理器打开
        try:
            if sys.platform == 'win32':
                os.startfile(final_output_dir)
            elif sys.platform == 'darwin':  # macOS
                os.system(f'open "{final_output_dir}"')
            else:  # Linux
                os.system(f'xdg-open "{final_output_dir}"')
        except Exception as e:
            QMessageBox.warning(self, "错误", f"打开目录失败: {str(e)}")
    
    def delete_result(self, row):
        """删除处理结果"""
        if row < 0 or row >= len(self.results):
            return
            
        result = self.results[row]
        if not result.get('success', False):
            return
            
        dir_name = result.get('dir_name', '')
        final_output_dir = os.path.join(self.output_dir, "final_output", dir_name)
        
        if not os.path.exists(final_output_dir):
            QMessageBox.warning(self, "错误", f"找不到输出目录: {final_output_dir}")
            return
            
        # 确认删除
        reply = QMessageBox.question(self, "确认删除", 
            f"确定要删除 {dir_name} 的处理结果吗？此操作不可撤销。",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            
        if reply != QMessageBox.Yes:
            return
            
        # 执行删除
        try:
            shutil.rmtree(final_output_dir)
            QMessageBox.information(self, "成功", f"已删除 {dir_name} 的处理结果")
            
            # 更新表格
            status_item = self.result_table.item(row, 1)
            status_item.setText("已删除")
            status_item.setForeground(QColor("gray"))
            
            # 禁用操作按钮
            cell_widget = self.result_table.cellWidget(row, 3)
            for i in range(cell_widget.layout().count()):
                widget = cell_widget.layout().itemAt(i).widget()
                if isinstance(widget, QPushButton):
                    widget.setEnabled(False)
            
            # 更新结果数据
            self.results[row]['success'] = False
            self.results[row]['frame_count'] = 0
        except Exception as e:
            QMessageBox.warning(self, "错误", f"删除失败: {str(e)}")
    
    def open_output_dir(self):
        """打开输出目录"""
        output_dir = os.path.join(self.output_dir, "final_output")
        if not os.path.exists(output_dir):
            QMessageBox.warning(self, "错误", f"找不到输出目录: {output_dir}")
            return
            
        try:
            if sys.platform == 'win32':
                os.startfile(output_dir)
            elif sys.platform == 'darwin':  # macOS
                os.system(f'open "{output_dir}"')
            else:  # Linux
                os.system(f'xdg-open "{output_dir}"')
        except Exception as e:
            QMessageBox.warning(self, "错误", f"打开目录失败: {str(e)}") 