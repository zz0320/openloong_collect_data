#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""ZIP压缩工作线程

处理数据压缩任务的异步线程
"""

import os
import shutil
from PyQt5.QtCore import QThread, pyqtSignal


class ZipWorker(QThread):
    """异步ZIP压缩工作线程"""
    progress_signal = pyqtSignal(int)
    finished_signal = pyqtSignal(str)
    error_signal = pyqtSignal(str)
    
    def __init__(self, data_storage, directory, compression_level=6, delete_after=False, output_dir=None):
        super().__init__()
        self.data_storage = data_storage
        self.directory = directory
        self.compression_level = compression_level
        self.delete_after = delete_after
        self.output_dir = output_dir  # 添加output_dir参数
        
    def run(self):
        try:
            # 设置save_dir确保压缩正确的目录
            self.data_storage.save_dir = self.directory
            
            # 创建ZIP，传入输出目录
            zip_path = self.data_storage.create_zip_archive(
                compression_level=self.compression_level,
                signal=self.progress_signal,
                output_dir=self.output_dir  # 传入输出目录
            )
            
            # 删除原始目录（如果需要）
            if zip_path and self.delete_after:
                try:
                    shutil.rmtree(self.directory)
                except Exception as e:
                    self.error_signal.emit(f"删除原始数据目录时出错: {str(e)}")
            
            if zip_path:
                self.finished_signal.emit(zip_path)
            else:
                self.error_signal.emit("创建ZIP归档失败")
        except Exception as e:
            self.error_signal.emit(f"压缩过程中出错: {str(e)}") 