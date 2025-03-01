#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""数据处理工作线程

处理数据同步和转换任务的异步线程
"""

import os
from PyQt5.QtCore import QThread, pyqtSignal
from concurrent.futures import ThreadPoolExecutor, as_completed
from ..core.data_processor import DataProcessor


class DataProcessWorker(QThread):
    """异步数据处理工作线程"""
    progress_signal = pyqtSignal(str)
    finished_signal = pyqtSignal(bool, list)  # 添加处理结果列表
    
    def __init__(self, data_dir, output_dir, max_workers=3, delete_failed=True):
        super().__init__()
        self.data_dir = data_dir
        self.output_dir = output_dir
        self.max_workers = max_workers
        self.delete_failed = delete_failed
        self.results = []  # 存储处理结果
        
    def run(self):
        try:
            self.progress_signal.emit(f"开始处理目录: {self.data_dir}")
            
            # 如果输入是单个目录
            if os.path.isdir(self.data_dir) and not any(os.path.isdir(os.path.join(self.data_dir, d)) for d in os.listdir(self.data_dir) if not d.startswith('.')):
                processor = DataProcessor(self.data_dir, self.output_dir)
                success = processor.process(self.progress_signal, self.delete_failed)
                if success:
                    stats = processor.get_result_stats()
                    self.results.append(stats)
                self.finished_signal.emit(success, self.results)
            else:
                # 如果输入是包含多个目录的基目录
                self.process_all_directories()
        except Exception as e:
            self.progress_signal.emit(f"处理过程中出错: {str(e)}")
            self.finished_signal.emit(False, self.results)
    
    def process_all_directories(self):
        os.makedirs(self.output_dir, exist_ok=True)
        
        # 获取输出目录的基本名称（不含路径）
        output_base_name = os.path.basename(self.output_dir)
        
        subdirs = [
            os.path.join(self.data_dir, d) 
            for d in os.listdir(self.data_dir) 
            if os.path.isdir(os.path.join(self.data_dir, d)) and 
            not d.startswith('.') and
            d != output_base_name  # 排除输出目录
        ]
        
        # 打印所有找到的目录名称
        self.progress_signal.emit("找到以下目录:")
        for i, subdir in enumerate(subdirs):
            self.progress_signal.emit(f"{i+1}. {os.path.basename(subdir)}")
        
        total_dirs = len(subdirs)
        self.progress_signal.emit(f"找到 {total_dirs} 个目录需要处理")
        successful = 0
        failed = 0
        
        with ThreadPoolExecutor(max_workers=self.max_workers) as executor:
            future_to_dir = {
                executor.submit(self.process_single_dir, d): d 
                for d in subdirs
            }
            
            for future in as_completed(future_to_dir):
                data_dir = future_to_dir[future]
                try:
                    dir_name, success, stats = future.result()
                    if success:
                        successful += 1
                        self.progress_signal.emit(f"成功处理: {dir_name}")
                        self.results.append(stats)
                    else:
                        failed += 1
                        self.progress_signal.emit(f"处理失败: {dir_name}")
                except Exception as e:
                    self.progress_signal.emit(f"处理 {data_dir} 时出错: {str(e)}")
                    failed += 1
        
        self.progress_signal.emit(f"\n===== 处理完成 =====")
        self.progress_signal.emit(f"总目录数: {total_dirs}")
        self.progress_signal.emit(f"成功: {successful}")
        self.progress_signal.emit(f"失败: {failed}")
        self.progress_signal.emit(f"结果保存到: {self.output_dir}")
        
        self.finished_signal.emit(successful > 0, self.results)
    
    def process_single_dir(self, d):
        processor = DataProcessor(d, self.output_dir)
        success = processor.process(self.progress_signal, self.delete_failed)
        stats = processor.get_result_stats() if success else {'success': False, 'dir_name': os.path.basename(d)}
        return d, success, stats 