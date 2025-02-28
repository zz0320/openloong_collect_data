#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 集成ROS数据记录器和数据处理器
# 主要功能:
# 1. 通过GUI界面记录ROS话题数据
# 2. 对记录的数据进行同步、转换与重组
# 3. 生成标准格式的数据集

import os
import time
import csv
import numpy as np
import argparse
import threading
import glob
import h5py
import pandas as pd
from collections import deque
from datetime import datetime
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
import rospy
import zipfile
import shutil
from tqdm import tqdm
from typing import List, Dict, Tuple, Any
from sensor_msgs.msg import Image, JointState
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
from mocap2robot.msg import vp_control
from sdk.msg import PuppetState, AlohaCmd
import sys
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                          QPushButton, QLabel, QTableWidget, QTableWidgetItem, QGroupBox, 
                          QProgressBar, QTextEdit, QTabWidget, QFileDialog, QGridLayout,
                          QSpacerItem, QSizePolicy, QCheckBox, QComboBox, QScrollArea,
                          QFrame, QHeaderView, QSplitter, QMessageBox, QListWidget,
                          QListWidgetItem, QDialog, QRadioButton, QButtonGroup, QLineEdit)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, pyqtSlot, QThread
from PyQt5.QtGui import QColor, QFont, QPixmap, QIcon, QFontDatabase, QTextCursor


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
        temp_sync_dir = os.path.join(self.output_dir, "temp_sync")
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
        
        # 清理临时目录
        try:
            if os.path.exists(temp_sync_dir):
                shutil.rmtree(temp_sync_dir)
                self.progress_signal.emit("已清理临时目录")
        except Exception as e:
            self.progress_signal.emit(f"警告: 清理临时目录失败: {str(e)}")
        
        self.finished_signal.emit(successful > 0, self.results)
    
    def process_single_dir(self, d):
        processor = DataProcessor(d, self.output_dir)
        success = processor.process(self.progress_signal, self.delete_failed)
        stats = processor.get_result_stats() if success else {'success': False, 'dir_name': os.path.basename(d)}
        return d, success, stats

class PlaybackThread(QThread):
    """轨迹回放线程"""
    progress_signal = pyqtSignal(int)
    text_signal = pyqtSignal(str)
    log_signal = pyqtSignal(str, bool, bool)
    completed_signal = pyqtSignal()
    image_signal = pyqtSignal(str, QPixmap)  # 新增：图像信号
    
    def __init__(self, hdf5_data, rate_hz, show_images=True, camera_base=None):
        super().__init__()
        self.data = hdf5_data
        self.rate_hz = rate_hz
        self.show_images = show_images
        self.camera_base = camera_base
        self.is_playing = True
        self.current_frame = 0
        self.total_frames = len(hdf5_data['timestamps']) if hdf5_data else 0
    
    def run(self):
        try:
            print("线程开始: 初始化阶段")
            # 初始化ROS节点已经在主线程完成
            
            # 创建发布器
            print("线程步骤1: 创建ROS发布器")
            aloha_cmd_pub = rospy.Publisher('/aloha_cmd', AlohaCmd, queue_size=10)
            gripper_action_pub = rospy.Publisher('/gripper_action_pub', JointState, queue_size=10)
            
            # 确保数据有效
            print("线程步骤2: 检查数据有效性")
            if not self.data or 'actions_joint' not in self.data:
                raise ValueError("数据无效或未包含必要的键")
            
            # 计算帧间时间间隔(毫秒)
            frame_interval_ms = int(1000.0 / self.rate_hz)
            print(f"线程步骤3: 设置回放帧率: {self.rate_hz}Hz (帧间隔 {frame_interval_ms}ms)")
            
            # 回放预告
            print(f"线程步骤4: 准备回放 {self.total_frames} 帧数据")
            self.log_signal.emit(f"准备回放 {self.total_frames} 帧数据", False, False)
            
            print("线程步骤5: 回放主循环")
            # 跟踪处理时间，用于实现精确的帧率控制
            last_frame_time = time.time()
            
            # 回放主循环
            for frame in range(self.total_frames):
                if not self.is_playing or self.isInterruptionRequested():
                    print(f"线程: 在第{frame}帧检测到停止请求")
                    break
                
                # 计算从上一帧开始经过的时间
                current_time = time.time()
                elapsed_since_last = current_time - last_frame_time
                
                # 如果处理太慢，可能需要跳过一些帧以维持目标帧率
                target_interval = 1.0 / self.rate_hz
                if elapsed_since_last > target_interval * 1.5 and frame > 0:
                    frames_to_skip = int(elapsed_since_last / target_interval) - 1
                    if frames_to_skip > 0:
                        frame += min(frames_to_skip, 5)  # 最多跳过5帧，防止跳过太多
                        print(f"线程: 为保持帧率跳过了 {frames_to_skip} 帧")
                        self.log_signal.emit(f"为保持帧率跳过了 {frames_to_skip} 帧", False, True)
                        
                        # 确保不会越界
                        if frame >= self.total_frames:
                            break
                
                print(f"线程: 处理第{frame}帧")
                
                # 发布机械臂命令
                try:
                    joint_cmd = self.data['actions_joint'][frame]
                    gripper_cmd = self.data['actions_gripper'][frame]
                    
                    left_joint_cmd = joint_cmd[0:7].tolist()
                    right_joint_cmd = joint_cmd[7:14].tolist()
                    
                    # 调试信息
                    print(f"发布第{frame}帧: 左臂={left_joint_cmd[:3]}, 右臂={right_joint_cmd[:3]}")
                    
                    # 创建控制消息
                    aloha_cmd = AlohaCmd()
                    aloha_cmd.arx_pos_left = left_joint_cmd
                    aloha_cmd.arx_pos_right = right_joint_cmd
                    aloha_cmd.cmd_left = 2
                    aloha_cmd.cmd_right = 2
                    
                    # 发布消息
                    print(f"线程: 发布机械臂命令")
                    aloha_cmd_pub.publish(aloha_cmd)
                    
                    # 发布夹爪命令
                    gripper_msg = JointState()
                    gripper_msg.header.stamp = rospy.Time.now()
                    gripper_msg.position = [float(gripper_cmd[0]), float(gripper_cmd[1])]
                    
                    print(f"线程: 发布夹爪命令")
                    gripper_action_pub.publish(gripper_msg)
                    
                except Exception as e:
                    print(f"线程: 发布命令错误: {str(e)}")
                    self.log_signal.emit(f"发布命令错误: {str(e)}", True, False)
                
                # 显示图像 - 使用Qt信号将图像发送到UI线程
                if self.show_images and self.camera_base:
                    try:
                        print(f"线程: 处理图像显示")
                        
                        # 构建当前帧的图像目录
                        frame_dir = os.path.join(self.camera_base, str(frame))
                        
                        # 检查目录是否存在
                        if os.path.exists(frame_dir):
                            # 加载并发送每个相机图像
                            for cam_type, display_name in [
                                ('front', '前方相机'),
                                ('left', '左侧相机'),
                                ('right', '右侧相机')
                            ]:
                                img_path = os.path.join(frame_dir, f"cam_{cam_type}_color.jpg")
                                if os.path.exists(img_path):
                                    # 使用Qt的QPixmap加载图像，避免OpenCV
                                    pixmap = QPixmap(img_path)
                                    # 调整大小以适应显示
                                    pixmap = pixmap.scaled(480, 360, Qt.KeepAspectRatio, Qt.SmoothTransformation)
                                    # 发送信号到UI线程显示图像
                                    self.image_signal.emit(display_name, pixmap)
                        else:
                            print(f"线程: 找不到图像目录 {frame_dir}")
                            self.log_signal.emit(f"警告: 找不到图像目录 {frame_dir}", True, False)
                    
                    except Exception as e:
                        print(f"线程: 图像处理错误: {str(e)}")
                        self.log_signal.emit(f"图像显示错误: {str(e)}", True, False)
                
                # 更新UI
                print(f"线程: 发送UI更新信号")
                self.current_frame = frame + 1
                self.progress_signal.emit(self.current_frame)
                self.text_signal.emit(f"{self.current_frame}/{self.total_frames}")
                
                if frame % 10 == 0:
                    self.log_signal.emit(
                        f"正在回放: 帧 {self.current_frame}/{self.total_frames}", 
                        False, True
                    )
                
                # 控制帧率 - 计算真正需要休眠的时间
                now = time.time()
                elapsed = now - last_frame_time
                sleep_time = max(0, frame_interval_ms / 1000.0 - elapsed)
                
                if sleep_time > 0:
                    print(f"线程: 等待下一帧 ({sleep_time*1000:.1f}ms)")
                    QThread.msleep(int(sleep_time * 1000))
                
                # 更新上一帧的时间戳
                last_frame_time = time.time()
                print(f"线程: 帧{frame}处理完成，继续下一帧")
            
            print(f"线程步骤6: 完成回放 ({self.current_frame}/{self.total_frames}帧)")
            if self.current_frame >= self.total_frames:
                self.log_signal.emit("回放完成!", False, False)
            else:
                self.log_signal.emit(f"回放中断，已播放 {self.current_frame}/{self.total_frames} 帧", False, False)
            
            self.completed_signal.emit()
            
        except Exception as e:
            print(f"线程错误: {e}")
            import traceback
            traceback.print_exc()
            self.log_signal.emit(f"回放过程中发生错误: {str(e)}", True, False)
        finally:
            print("线程正常结束")

class TrajectoryPlayerDialog(QDialog):
    """轨迹回放对话框"""
    def __init__(self, hdf5_path=None, parent=None):
        super().__init__(parent)
        self.hdf5_path = hdf5_path
        self.is_playing = False
        self.current_frame = 0
        self.total_frames = 0
        self.data = None
        self.playback_thread = None
        self.camera_base = None
        self.init_ui()
        
    def init_ui(self):
        """初始化用户界面"""
        self.setWindowTitle("轨迹回放控制器")
        self.setMinimumSize(1000, 700)  # 增大窗口以容纳图像显示
        
        layout = QVBoxLayout(self)
        
        # 文件信息区域
        file_group = QGroupBox("文件信息")
        file_layout = QGridLayout(file_group)
        
        file_layout.addWidget(QLabel("HDF5文件:"), 0, 0)
        self.file_path_label = QLabel(self.hdf5_path or "未选择文件")
        self.file_path_label.setWordWrap(True)
        file_layout.addWidget(self.file_path_label, 0, 1, 1, 2)
        
        browse_button = QPushButton("浏览...")
        browse_button.clicked.connect(self.browse_file)
        file_layout.addWidget(browse_button, 0, 3)
        
        file_layout.addWidget(QLabel("数据信息:"), 1, 0)
        self.data_info_label = QLabel("未加载")
        file_layout.addWidget(self.data_info_label, 1, 1, 1, 3)
        
        layout.addWidget(file_group)
        
        # 图像显示区域 (新增)
        image_group = QGroupBox("相机图像")
        image_layout = QHBoxLayout(image_group)
        
        # 三个图像标签，用于显示相机图像
        self.image_labels = {}
        for cam_name in ['前方相机', '左侧相机', '右侧相机']:
            cam_frame = QFrame()
            cam_frame.setFrameShape(QFrame.Box)
            cam_frame.setLineWidth(1)
            cam_layout = QVBoxLayout(cam_frame)
            
            # 标题
            title_label = QLabel(cam_name)
            title_label.setAlignment(Qt.AlignCenter)
            cam_layout.addWidget(title_label)
            
            # 图像标签
            img_label = QLabel()
            img_label.setAlignment(Qt.AlignCenter)
            img_label.setMinimumSize(320, 240)
            img_label.setText("无图像")
            cam_layout.addWidget(img_label)
            
            image_layout.addWidget(cam_frame)
            self.image_labels[cam_name] = img_label
        
        layout.addWidget(image_group)
        
        # 回放控制区域
        control_group = QGroupBox("回放控制")
        control_layout = QVBoxLayout(control_group)
        
        # 进度条
        progress_layout = QHBoxLayout()
        progress_layout.addWidget(QLabel("进度:"))
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        progress_layout.addWidget(self.progress_bar)
        self.frame_label = QLabel("0/0")
        progress_layout.addWidget(self.frame_label)
        control_layout.addLayout(progress_layout)
        
        # 按钮区域
        buttons_layout = QHBoxLayout()
        
        self.play_button = QPushButton("播放")
        self.play_button.setIcon(QIcon.fromTheme("media-playback-start"))
        self.play_button.clicked.connect(self.toggle_play)
        self.play_button.setEnabled(False)
        buttons_layout.addWidget(self.play_button)
        
        self.stop_button = QPushButton("停止")
        self.stop_button.setIcon(QIcon.fromTheme("media-playback-stop"))
        self.stop_button.clicked.connect(self.stop_playback)
        self.stop_button.setEnabled(False)
        buttons_layout.addWidget(self.stop_button)
        
        self.load_button = QPushButton("加载数据")
        self.load_button.clicked.connect(self.load_data)
        self.load_button.setEnabled(bool(self.hdf5_path))
        buttons_layout.addWidget(self.load_button)
        
        control_layout.addLayout(buttons_layout)
        
        # 回放选项
        options_layout = QHBoxLayout()
        options_layout.addWidget(QLabel("回放速率:"))
        
        self.rate_combo = QComboBox()
        rates = ["25Hz", "30Hz", "50Hz", "60Hz", "100Hz"]
        self.rate_combo.addItems(rates)
        self.rate_combo.setCurrentText("60Hz")
        options_layout.addWidget(self.rate_combo)
        
        options_layout.addWidget(QLabel("显示图像:"))
        self.show_images_check = QCheckBox()
        self.show_images_check.setChecked(True)
        options_layout.addWidget(self.show_images_check)
        
        control_layout.addLayout(options_layout)
        
        layout.addWidget(control_group)
        
        # 日志区域
        log_group = QGroupBox("回放日志")
        log_layout = QVBoxLayout(log_group)
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        log_layout.addWidget(self.log_text)
        
        layout.addWidget(log_group)
        
        # 底部按钮
        buttons_layout = QHBoxLayout()
        close_button = QPushButton("关闭")
        close_button.clicked.connect(self.close)
        buttons_layout.addWidget(close_button)
        
        layout.addLayout(buttons_layout)
    
    def browse_file(self):
        """浏览并选择HDF5文件"""
        if self.is_playing:
            self.log_message("请先停止回放再更换文件", error=True)
            return
            
        file_path, _ = QFileDialog.getOpenFileName(
            self, "选择HDF5文件", "", "HDF5文件 (*.h5 *.hdf5);;所有文件 (*.*)"
        )
        
        if file_path:
            self.hdf5_path = file_path
            self.file_path_label.setText(file_path)
            self.load_button.setEnabled(True)
            self.log_message(f"已选择文件: {file_path}")
    
    def load_data(self):
        """加载HDF5数据"""
        if not self.hdf5_path or not os.path.exists(self.hdf5_path):
            self.log_message("请先选择有效的HDF5文件", error=True)
            return
            
        try:
            self.log_message("正在加载数据...")
            
            # 加载HDF5文件
            with h5py.File(self.hdf5_path, 'r') as f:
                # 获取动作数据
                actions_joint = f['action/joint/position'][:]
                actions_gripper = f['action/effector/position'][:]
                
                # 获取状态数据
                states_joint = f['state/joint/position'][:]
                states_gripper = f['state/effector/position'][:]
                
                # 获取时间戳
                timestamps = f['timestamp'][:]
                
                # 保存数据
                self.data = {
                    'actions_joint': actions_joint,
                    'actions_gripper': actions_gripper,
                    'states_joint': states_joint,
                    'states_gripper': states_gripper,
                    'timestamps': timestamps
                }
                # 添加数据验证日志
                self.log_message(f"读取动作数据: 形状={actions_joint.shape}, 范围=[{actions_joint.min()}, {actions_joint.max()}]")
                self.log_message(f"读取夹爪数据: 形状={actions_gripper.shape}, 范围=[{actions_gripper.min()}, {actions_gripper.max()}]")
                            
            # 计算总帧数
            self.total_frames = len(self.data['timestamps'])
            
            # 检查图像帧数
            self.data_dir = os.path.dirname(os.path.dirname(self.hdf5_path))
            self.camera_base = os.path.join(self.data_dir, 'camera')
            
            if os.path.exists(self.camera_base):
                available_frames = len([d for d in os.listdir(self.camera_base) 
                                     if os.path.isdir(os.path.join(self.camera_base, d))])
                if available_frames != self.total_frames:
                    self.log_message(f"警告: 图像帧数 ({available_frames}) "
                                   f"与轨迹帧数 ({self.total_frames}) 不匹配", error=True)
            else:
                self.log_message("警告: 未找到对应的相机图像目录", error=True)
            
            # 更新UI
            self.data_info_label.setText(f"总帧数: {self.total_frames}, "
                                        f"动作维度: {actions_joint.shape}")
            self.progress_bar.setRange(0, self.total_frames)
            self.progress_bar.setValue(0)
            self.frame_label.setText(f"0/{self.total_frames}")
            
            # 启用控制按钮
            self.play_button.setEnabled(True)
            
            self.log_message(f"数据加载成功 - 总帧数: {self.total_frames}")
            
        except Exception as e:
            self.log_message(f"加载数据失败: {str(e)}", error=True)
    
    def toggle_play(self):
        """开始或暂停回放"""
        if not self.data:
            self.log_message("请先加载数据", error=True)
            return
            
        if self.is_playing:
            # 暂停回放
            if self.playback_thread:
                self.playback_thread.is_playing = False
            self.is_playing = False
            self.play_button.setText("继续")
            self.log_message("回放已暂停")
        else:
            # 开始/继续回放
            self.is_playing = True
            self.play_button.setText("暂停")
            self.stop_button.setEnabled(True)
            
            # 如果已经到达末尾，重新开始
            if self.current_frame >= self.total_frames:
                self.current_frame = 0
                self.progress_bar.setValue(0)
                self.frame_label.setText(f"0/{self.total_frames}")
            
            # 使用QThread替代Python线程
            if not self.playback_thread or not self.playback_thread.isRunning():
                # 获取回放速率
                rate_text = self.rate_combo.currentText()
                hz = int(rate_text.replace("Hz", ""))
                
                # 创建并启动线程
                self.playback_thread = PlaybackThread(
                    self.data, 
                    hz, 
                    self.show_images_check.isChecked(),
                    self.camera_base
                )
                
                # 连接信号
                self.playback_thread.progress_signal.connect(self.progress_bar.setValue)
                self.playback_thread.text_signal.connect(self.frame_label.setText)
                self.playback_thread.log_signal.connect(self.log_message)
                self.playback_thread.completed_signal.connect(self.playback_completed)
                # 新增：连接图像信号
                self.playback_thread.image_signal.connect(self.update_image)
                
                # 设置初始帧
                self.playback_thread.current_frame = self.current_frame
                
                # 启动线程
                self.playback_thread.start()
                
            self.log_message("开始回放")
    
    def stop_playback(self):
        """停止回放"""
        if self.playback_thread and self.playback_thread.isRunning():
            self.playback_thread.is_playing = False
            self.playback_thread.requestInterruption()
            self.playback_thread.wait(1000)  # 等待线程结束，最多1秒
            
        self.is_playing = False
        self.current_frame = 0
        self.progress_bar.setValue(0)
        self.frame_label.setText(f"0/{self.total_frames}")
        self.play_button.setText("播放")
        self.stop_button.setEnabled(False)
        
        # 清空图像显示
        for img_label in self.image_labels.values():
            img_label.setText("无图像")
            img_label.setPixmap(QPixmap())
        
        self.log_message("回放已停止")
    
    @pyqtSlot()
    def playback_completed(self):
        """回放完成处理"""
        self.is_playing = False
        self.play_button.setText("播放")
        self.stop_button.setEnabled(False)
        self.current_frame = self.total_frames
    
    @pyqtSlot(str, QPixmap)
    def update_image(self, camera_name, pixmap):
        """更新相机图像"""
        if camera_name in self.image_labels:
            self.image_labels[camera_name].setPixmap(pixmap)
    
    def log_message(self, message, error=False, update=False):
        """添加日志消息"""
        timestamp = datetime.now().strftime('%H:%M:%S')
        
        if error:
            log_entry = f"<span style='color:red'>[{timestamp}] ERROR: {message}</span>"
        else:
            log_entry = f"[{timestamp}] {message}"
        
        # 对于更新类型的消息，替换最后一行
        if update and self.log_text.document().blockCount() > 0:
            cursor = self.log_text.textCursor()
            cursor.movePosition(cursor.End)
            cursor.movePosition(cursor.StartOfBlock, cursor.KeepAnchor)
            cursor.removeSelectedText()
            cursor.deletePreviousChar()  # 删除换行符
            self.log_text.setTextCursor(cursor)
            self.log_text.append(log_entry)
        else:
            self.log_text.append(log_entry)
        
        # 滚动到底部
        self.log_text.verticalScrollBar().setValue(self.log_text.verticalScrollBar().maximum())
        
        # 同时在控制台输出
        print(f"[{timestamp}] {'ERROR: ' if error else ''}{message}")
    
    def closeEvent(self, event):
        """处理窗口关闭事件"""
        # 停止回放线程
        if self.playback_thread and self.playback_thread.isRunning():
            self.playback_thread.is_playing = False
            self.playback_thread.requestInterruption()
            if not self.playback_thread.wait(2000):  # 等待2秒
                print("警告: 无法正常终止回放线程")
        
        # 接受关闭事件
        event.accept()


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

class DataProcessor:
    """数据处理器，对记录的数据进行同步、转换与重组"""
    
    def __init__(self, data_dir: str, output_base: str):
        self.data_dir = data_dir
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.dir_name = os.path.basename(data_dir)
        
        self.sync_base = os.path.join(output_base, "temp_sync", f"{timestamp}_{self.dir_name}")
        self.sync_dir = os.path.join(self.sync_base, 'final_synchronized')
        self.final_output_dir = os.path.join(output_base, "final_output", self.dir_name)
        self.success = False  # 处理是否成功的标志
        
        os.makedirs(self.sync_dir, exist_ok=True)
        for cam in ['front', 'left', 'right']:
            os.makedirs(os.path.join(self.sync_dir, f'camera_{cam}'), exist_ok=True)
        
        os.makedirs(os.path.join(self.final_output_dir, 'camera'), exist_ok=True)
        os.makedirs(os.path.join(self.final_output_dir, 'record'), exist_ok=True)

    def _get_camera_files(self, camera_name: str) -> List[Tuple[str, float]]:
        image_dir = os.path.join(self.data_dir, f'camera_{camera_name}')
        image_files = glob.glob(os.path.join(image_dir, '*.jpg'))
        files_with_timestamps = [
            (f, float(os.path.basename(f).replace('.jpg', '')))
            for f in image_files
        ]
        return sorted(files_with_timestamps, key=lambda x: x[1])

    def _get_frame_counts(self) -> Dict[str, int]:
        counts = {}
        for cam in ['front', 'left', 'right']:
            counts[cam] = len(self._get_camera_files(cam))
        return counts
    
    def _find_closest_file(self, files: List[Tuple[str, float]], target_time: float) -> Tuple[str, float]:
        closest_idx = min(range(len(files)), key=lambda i: abs(files[i][1] - target_time))
        return files[closest_idx]

    def _process_joint_and_gripper_data(self, df: pd.DataFrame, is_action: bool = False) -> Tuple[np.ndarray, np.ndarray]:
        arm_prefix = 'left_arm_exp_' if is_action else 'left_arm_'
        gripper_prefix = 'left_gripper_exp' if is_action else 'left_gripper'
        
        left_arm_cols = [f'{arm_prefix}{i}' for i in range(1, 8)]
        left_arm_data = df[left_arm_cols].values
        
        right_arm_prefix = 'right_arm_exp_' if is_action else 'right_arm_'
        right_arm_cols = [f'{right_arm_prefix}{i}' for i in range(1, 8)]
        right_arm_data = df[right_arm_cols].values
        
        joint_data = np.concatenate([left_arm_data, right_arm_data], axis=1)
        
        left_gripper = df[gripper_prefix].values.reshape(-1, 1)
        right_gripper_prefix = 'right_gripper_exp' if is_action else 'right_gripper'
        right_gripper = df[right_gripper_prefix].values.reshape(-1, 1)
        gripper_data = np.concatenate([left_gripper, right_gripper], axis=1)
        
        return joint_data, gripper_data

    def _save_synchronized_data(self, valid_times, synced_states, synced_actions):
        states_output = []
        for time, state in zip(valid_times, synced_states):
            row = [time]
            for arm in ['left', 'right']:
                for i in range(1, 8):
                    row.append(state[f'{arm}_arm_{i}'])
                row.append(state[f'{arm}_gripper'])
            states_output.append(row)

        states_path = os.path.join(self.sync_dir, 'states_sync.csv')
        with open(states_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp',
                'left_arm_1', 'left_arm_2', 'left_arm_3', 'left_arm_4',
                'left_arm_5', 'left_arm_6', 'left_arm_7', 'left_gripper',
                'right_arm_1', 'right_arm_2', 'right_arm_3', 'right_arm_4',
                'right_arm_5', 'right_arm_6', 'right_arm_7', 'right_gripper'
            ])
            writer.writerows(states_output)

        actions_output = []
        for time, action in zip(valid_times, synced_actions):
            row = [time]
            for arm in ['left', 'right']:
                for i in range(1, 8):
                    row.append(action[f'{arm}_arm_exp_{i}'])
                row.append(action[f'{arm}_gripper_exp'])
            actions_output.append(row)

        actions_path = os.path.join(self.sync_dir, 'actions_sync.csv')
        with open(actions_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp',
                'left_arm_exp_1', 'left_arm_exp_2', 'left_arm_exp_3', 'left_arm_exp_4',
                'left_arm_exp_5', 'left_arm_exp_6', 'left_arm_exp_7', 'left_gripper_exp',
                'right_arm_exp_1', 'right_arm_exp_2', 'right_arm_exp_3', 'right_arm_exp_4',
                'right_arm_exp_5', 'right_arm_exp_6', 'right_arm_exp_7', 'right_gripper_exp'
            ])
            writer.writerows(actions_output)

    def step1_synchronize(self, progress_signal=None) -> bool:
        try:
            if progress_signal:
                progress_signal.emit(f"\n步骤1: 同步来自 {self.data_dir} 的数据")
            
            frame_counts = self._get_frame_counts()
            min_cam = min(frame_counts.items(), key=lambda x: x[1])[0]
            min_frames = frame_counts[min_cam]
            
            if progress_signal:
                progress_signal.emit(f"帧数统计: {frame_counts}")
                progress_signal.emit(f"使用 {min_cam} 相机作为参考，共 {min_frames} 帧")
            
            # 如果帧数太少，可能不是有效数据
            if min_frames < 5:
                if progress_signal:
                    progress_signal.emit(f"警告: 有效帧数太少 ({min_frames}), 可能无法正确同步")
                if min_frames == 0:
                    if progress_signal:
                        progress_signal.emit(f"错误: 没有有效帧，同步失败")
                    return False
            
            camera_files = {
                cam: self._get_camera_files(cam)
                for cam in ['front', 'left', 'right']
            }

            ref_camera_files = camera_files[min_cam][:min_frames]
            ref_times = [time for _, time in ref_camera_files]

            if progress_signal:
                progress_signal.emit("正在同步图像...")
            
            for cam_name in ['front', 'left', 'right']:
                if progress_signal:
                    progress_signal.emit(f"处理 {cam_name} 相机...")
                
                files = camera_files[cam_name]
                for ref_time in tqdm(ref_times):
                    src_file, _ = self._find_closest_file(files, ref_time)
                    dst_name = f"{ref_time:.6f}.jpg"
                    dst_path = os.path.join(self.sync_dir, f'camera_{cam_name}', dst_name)
                    shutil.copy2(src_file, dst_path)

            if progress_signal:
                progress_signal.emit("正在同步状态和动作数据...")
            
            # 检查CSV文件是否存在
            states_path = os.path.join(self.data_dir, 'csv/states.csv')
            actions_path = os.path.join(self.data_dir, 'csv/actions.csv')
            
            if not os.path.exists(states_path) or not os.path.exists(actions_path):
                if progress_signal:
                    progress_signal.emit(f"错误: 缺少必要的CSV文件")
                return False
                
            states_df = pd.read_csv(states_path)
            actions_df = pd.read_csv(actions_path)

            # 检查数据帧是否足够
            if len(states_df) < 5 or len(actions_df) < 5:
                if progress_signal:
                    progress_signal.emit(f"错误: 状态或动作数据不足")
                return False

            synced_states = []
            synced_actions = []

            for ref_time in tqdm(ref_times):
                state_idx = (states_df['timestamp'] - ref_time).abs().argmin()
                action_idx = (actions_df['timestamp'] - ref_time).abs().argmin()
                
                synced_states.append(states_df.iloc[state_idx])
                synced_actions.append(actions_df.iloc[action_idx])

            if progress_signal:
                progress_signal.emit("保存同步数据...")
            
            self._save_synchronized_data(ref_times, synced_states, synced_actions)
            
            return True

        except Exception as e:
            if progress_signal:
                progress_signal.emit(f"同步过程中出错: {str(e)}")
            return False

    def step2_convert_to_h5(self, progress_signal=None) -> bool:
        try:
            if progress_signal:
                progress_signal.emit("\n步骤2: 转换数据为HDF5格式")
            
            states_path = os.path.join(self.sync_dir, 'states_sync.csv')
            actions_path = os.path.join(self.sync_dir, 'actions_sync.csv')
            
            # 检查同步后的CSV文件是否存在
            if not os.path.exists(states_path) or not os.path.exists(actions_path):
                if progress_signal:
                    progress_signal.emit("错误: 找不到同步后的CSV文件")
                return False
                
            states_df = pd.read_csv(states_path)
            actions_df = pd.read_csv(actions_path)
            
            # 检查数据是否为空
            if states_df.empty or actions_df.empty:
                if progress_signal:
                    progress_signal.emit("错误: 同步后的数据为空")
                return False
                
            h5_path = os.path.join(self.sync_dir, 'final_synchronized.h5')
            n_samples = len(states_df)
            
            if progress_signal:
                progress_signal.emit(f"创建HDF5文件，共 {n_samples} 个样本...")
            
            with h5py.File(h5_path, 'w') as h5file:
                # 1. 时间戳

                # 更精确的时间戳转换
                timestamps = states_df['timestamp'].values
                # 使用精确的算法将秒转换为纳秒
                timestamps_ns = np.array([int((ts * 1e9) + 0.5) for ts in timestamps], dtype=np.int64)
                h5file.create_dataset('timestamp', data=timestamps_ns, dtype=np.int64)

                indices = np.arange(n_samples)

                # 2. 处理从CSV得到的数据
                state_joint_data, state_gripper_data = self._process_joint_and_gripper_data(states_df, is_action=False)
                action_joint_data, action_gripper_data = self._process_joint_and_gripper_data(actions_df, is_action=True)
                
                # State相关数据集
                state_datasets = {
                    'state/joint/position': (state_joint_data, np.float64),
                    'state/effector/position': (state_gripper_data, np.float64),
                    'state/joint/current_value': (state_joint_data, np.float64),
                }

                # Action相关数据集
                action_datasets = {
                    'action/joint/position': (action_joint_data, np.float64),
                    'action/effector/position': (action_gripper_data, np.float64),
                }

                # Index相关数据集
                for path in ['action/effector/index', 'action/end/index', 'action/head/index',
                            'action/joint/index', 'action/robot/index', 'action/waist/index']:
                    h5file.create_dataset(path, data=indices, dtype=np.int64)

                # 创建所有数据集
                for name, (data, dtype) in state_datasets.items():
                    if data is None:
                        h5file.create_dataset(name, shape=(0,), dtype=dtype)
                    else:
                        h5file.create_dataset(name, data=data, dtype=dtype)

                for name, (data, dtype) in action_datasets.items():
                    if data is None:
                        h5file.create_dataset(name, shape=(0,), dtype=dtype)
                    else:
                        h5file.create_dataset(name, data=data, dtype=dtype)
                
                # 创建空数据集
                self._create_empty_dataset(h5file)
                
                if progress_signal:
                    progress_signal.emit(f"HDF5文件已创建: {h5_path}")
                
                return True
            
        except Exception as e:
            if progress_signal:
                progress_signal.emit(f"转换为HDF5时出错: {str(e)}")
            return False

    def _create_empty_dataset(self, h5file: h5py.File):
        # 从CSV来的数据已经处理了，只需创建其他字段为空
        empty_fields = [
            'action/effector/force',    # 末端执行器力
            'action/joint/effort',      # 关节力矩
            'action/joint/velocity',    # 关节速度
            'action/robot/orientation', # 机器人朝向
            'action/robot/position',    # 机器人位置
            'action/end/orientation',   # 末端执行器朝向 
            'action/end/position',      # 末端执行器位置
            'action/head/position',     # 头部位置
            'action/robot/velocity',    # 机器人速度
            'action/waist/position',    # 腰部位置
            'state/effector/force',     # 末端执行器力
            'state/end/angular',        # 末端角速度
            'state/end/velocity',       # 末端线速度 
            'state/end/wrench',         # 末端扭矩
            'state/head/effort',        # 头部力矩
            'state/head/velocity',      # 头部速度
            'state/joint/effort',       # 关节力矩
            'state/joint/velocity',     # 关节速度
            'state/robot/orientation_drift', # 机器人朝向漂移
            'state/robot/position_drift',    # 机器人位置漂移
            'state/waist/effort',       # 腰部力矩
            'state/waist/velocity',     # 腰部速度
            'state/end/orientation',    # 末端朝向
            'state/end/position',       # 末端位置
            'state/head/position',      # 头部位置
            'state/robot/orientation',  # 机器人朝向
            'state/robot/position',     # 机器人位置 
            'state/waist/position'      # 腰部位置
        ]
        
        for field in empty_fields:
            h5file.create_dataset(field, shape=(0,), dtype=np.float32)

    def step3_reorganize(self, progress_signal=None) -> bool:
        try:
            if progress_signal:
                progress_signal.emit("\n步骤3: 重新组织数据结构")
            
            cam_folders = ["camera_front", "camera_left", "camera_right"]
            h5_path = os.path.join(self.sync_dir, 'final_synchronized.h5')
            
            # 检查HDF5文件是否存在
            if not os.path.exists(h5_path):
                if progress_signal:
                    progress_signal.emit("错误: 找不到HDF5文件")
                return False
                
            # 检查相机文件夹是否都存在
            for cam_folder in cam_folders:
                if not os.path.exists(os.path.join(self.sync_dir, cam_folder)):
                    if progress_signal:
                        progress_signal.emit(f"错误: 找不到相机文件夹 {cam_folder}")
                    return False
            
            image_dict = {}
            for cam_folder in cam_folders:
                cam_path = os.path.join(self.sync_dir, cam_folder)
                images = sorted(os.listdir(cam_path))
                if not images:
                    if progress_signal:
                        progress_signal.emit(f"错误: 相机文件夹 {cam_folder} 中没有图像")
                    return False
                image_dict[cam_folder] = images

            min_length = min(len(images) for images in image_dict.values())
            if progress_signal:
                progress_signal.emit(f"处理 {min_length} 帧...")
            
            # 检查是否有足够的帧
            if min_length < 5:
                if progress_signal:
                    progress_signal.emit(f"错误: 图像帧数不足 ({min_length})")
                return False
            
            for idx in tqdm(range(min_length)):
                folder_name = os.path.join(self.final_output_dir, 'camera', f"{idx}")
                os.makedirs(folder_name, exist_ok=True)
                
                for cam_folder in cam_folders:
                    src_image = os.path.join(self.sync_dir, cam_folder, image_dict[cam_folder][idx])
                    ext = os.path.splitext(image_dict[cam_folder][idx])[-1]
                    dst_name = f"cam_{cam_folder.split('_')[1]}_color{ext}"
                    dst_image = os.path.join(folder_name, dst_name)
                    shutil.copy2(src_image, dst_image)

            h5_dest = os.path.join(self.final_output_dir, 'record', 'aligned_joints.h5')
            shutil.copy2(h5_path, h5_dest)
            
            # 验证结果
            if not os.path.exists(h5_dest):
                if progress_signal:
                    progress_signal.emit("错误: 无法复制HDF5文件到最终目录")
                return False
                
            # 随机检查一些图像是否正确复制
            for cam_folder in cam_folders:
                cam_type = cam_folder.split('_')[1]
                check_idx = min(3, min_length - 1)
                check_path = os.path.join(self.final_output_dir, 'camera', f"{check_idx}", f"cam_{cam_type}_color.jpg")
                if not os.path.exists(check_path):
                    if progress_signal:
                        progress_signal.emit(f"错误: 缺少图像文件 {check_path}")
                    return False
            
            if progress_signal:
                progress_signal.emit(f"数据已重组到: {self.final_output_dir}")
                progress_signal.emit(f"总帧数: {min_length}")
            
            # 创建处理成功标记文件
            with open(os.path.join(self.final_output_dir, 'SUCCESS'), 'w') as f:
                f.write(f"处理成功时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"总帧数: {min_length}\n")
                f.write(f"源目录: {self.data_dir}\n")
            
            self.success = True
            return True
            
        except Exception as e:
            if progress_signal:
                progress_signal.emit(f"重组过程中出错: {str(e)}")
            return False

    def cleanup(self, progress_signal=None, delete_failed=True):
        """清理临时文件，如果处理失败且delete_failed为True，则同时删除输出目录"""
        try:
            # 删除临时文件夹
            if os.path.exists(self.sync_base):
                shutil.rmtree(self.sync_base)
                if progress_signal:
                    progress_signal.emit("临时文件已清理")
            
            # 如果处理失败且delete_failed为True，删除输出目录
            if not self.success and delete_failed and os.path.exists(self.final_output_dir):
                shutil.rmtree(self.final_output_dir)
                if progress_signal:
                    progress_signal.emit(f"已删除失败的输出目录: {self.final_output_dir}")
        except Exception as e:
            if progress_signal:
                progress_signal.emit(f"警告: 清理文件失败: {str(e)}")

    def process(self, progress_signal=None, delete_failed=True) -> bool:
        """执行完整的处理流程"""
        try:
            self.success = False  # 重置成功标志
            
            if not self.step1_synchronize(progress_signal):
                if progress_signal:
                    progress_signal.emit(f"同步失败，终止处理")
                self.cleanup(progress_signal, delete_failed)
                return False
            
            if not self.step2_convert_to_h5(progress_signal):
                if progress_signal:
                    progress_signal.emit(f"HDF5转换失败，终止处理")
                self.cleanup(progress_signal, delete_failed)
                return False
            
            if not self.step3_reorganize(progress_signal):
                if progress_signal:
                    progress_signal.emit(f"重组数据失败，终止处理")
                self.cleanup(progress_signal, delete_failed)
                return False
            
            # 所有步骤都成功
            self.cleanup(progress_signal, delete_failed=False)  # 只清理临时文件
            return True
            
        except Exception as e:
            if progress_signal:
                progress_signal.emit(f"处理过程中出错: {str(e)}")
            self.cleanup(progress_signal, delete_failed)
            return False

    def get_result_stats(self) -> Dict:
        """获取处理结果的统计信息"""
        stats = {
            'success': self.success,
            'dir_name': self.dir_name,
            'frame_count': 0,
            'image_size': 0
        }
        
        if not self.success or not os.path.exists(self.final_output_dir):
            return stats
            
        # 计算帧数
        camera_dir = os.path.join(self.final_output_dir, 'camera')
        if os.path.exists(camera_dir):
            frame_dirs = [d for d in os.listdir(camera_dir) if os.path.isdir(os.path.join(camera_dir, d))]
            stats['frame_count'] = len(frame_dirs)
            
            # 获取图像大小
            if frame_dirs:
                sample_frame = os.path.join(camera_dir, frame_dirs[0])
                sample_image = os.path.join(sample_frame, 'cam_front_color.jpg')
                if os.path.exists(sample_image):
                    stats['image_size'] = os.path.getsize(sample_image) / 1024  # KB
        
        return stats

class TopicMonitor:
    """话题状态监控类，负责追踪ROS话题的状态"""
    
    def __init__(self, timeout=2.0):
        self.topics = {
            'front_camera': TopicState(),
            'left_camera': TopicState(),
            'right_camera': TopicState(),
            'puppet_state': TopicState(),
            'gripper_left': TopicState(),
            'gripper_right': TopicState(),
            'gripper_action': TopicState()
        }
        self.timeout = timeout
        
    def update_topic(self, topic_name):
        """更新话题状态信息"""
        if topic_name in self.topics:
            self.topics[topic_name].update()

    def get_topic_status(self, topic_name):
        """获取单个话题的状态"""
        if topic_name in self.topics:
            return self.topics[topic_name].get_status(self.timeout)
        return 'UNKNOWN'
    
    def all_topics_active(self):
        """检查所有话题是否都处于活跃状态"""
        # 定义必须要活跃的核心话题和可选话题
        core_topics = ['front_camera', 'left_camera', 'right_camera', 'puppet_state', 'gripper_action']
        
        # 检查核心话题是否都活跃
        for topic_name in core_topics:
            status = self.get_topic_status(topic_name)
            if status != 'ACTIVE':
                return False
        
        # 可选话题不影响录制
        return True
    
    def reset_counts(self):
        """重置所有话题的消息计数"""
        for topic in self.topics.values():
            topic.count = 0


class TopicState:
    """话题状态数据类"""
    
    def __init__(self):
        self.last_time = 0
        self.freq = 0
        self.count = 0
        self.status = 'WAITING'  # WAITING, ACTIVE, TIMEOUT
        
    def update(self):
        """更新话题状态信息"""
        now = time.time()
        
        # 更新计数和频率
        self.count += 1
        if self.last_time > 0:
            dt = now - self.last_time
            self.freq = 1.0 / dt if dt > 0 else 0
        
        self.last_time = now
        self.status = 'ACTIVE'
        
    def get_status(self, timeout=2.0):
        """获取话题状态"""
        now = time.time()
        
        if self.last_time == 0:
            return 'WAITING'
        elif now - self.last_time > timeout:
            return 'TIMEOUT'
        else:
            return 'ACTIVE'

class DataStorage:
    """数据存储类，负责管理录制数据的保存和压缩"""
    
    def __init__(self, base_dir="./recorded_data", max_workers=4, args=None):
        self.base_dir = base_dir
        self.save_dir = None
        self.img_dirs = {}
        self.csv_dir = None
        self.recording_start_time = 0
        self.recording_duration = 0
        self.states = []
        self.actions = []
        self.bridge = CvBridge()
        self.executor = ThreadPoolExecutor(max_workers=max_workers)
        self.image_tasks = []
        self.args = args  # 保存args引用
        
        # 创建基本目录
        os.makedirs(self.base_dir, exist_ok=True)

    
    def start_new_recording(self):
        """开始新的录制，创建目录结构"""
        try:
            # 创建基础目录，带有时间戳以区分不同录制
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.save_dir = os.path.join(self.base_dir, timestamp)
            
            # 确保基本目录存在
            os.makedirs(self.base_dir, exist_ok=True)
            
            # 创建主目录
            if not os.path.exists(self.save_dir):
                os.makedirs(self.save_dir)
            
            # 创建图像目录
            self.img_dirs = {}
            for cam_name in ['front', 'left', 'right']:
                cam_dir = os.path.join(self.save_dir, f'camera_{cam_name}')
                if not os.path.exists(cam_dir):
                    os.makedirs(cam_dir)
                self.img_dirs[cam_name] = cam_dir
                
            # 创建CSV目录
            self.csv_dir = os.path.join(self.save_dir, 'csv')
            if not os.path.exists(self.csv_dir):
                os.makedirs(self.csv_dir)
            
            # 重置数据和时间
            self.states = []
            self.actions = []
            self.recording_start_time = time.time()
            self.recording_duration = 0
            
            return self.save_dir
        except Exception as e:
            rospy.logerr(f"创建录制目录结构时出错: {str(e)}")
            # 创建临时目录作为备选
            temp_dir = os.path.join(os.path.expanduser("~"), "ros_recorder_data", timestamp)
            rospy.logwarn(f"尝试创建备选目录: {temp_dir}")
            os.makedirs(temp_dir, exist_ok=True)
            
            # 设置备选目录结构
            self.save_dir = temp_dir
            for cam_name in ['front', 'left', 'right']:
                self.img_dirs[cam_name] = os.path.join(temp_dir, f'camera_{cam_name}')
                os.makedirs(self.img_dirs[cam_name], exist_ok=True)
            self.csv_dir = os.path.join(temp_dir, 'csv')
            os.makedirs(self.csv_dir, exist_ok=True)
            
            return self.save_dir
    
    def reset(self):
        """重置存储状态"""
        self.save_dir = None
        self.img_dirs = {}
        self.csv_dir = None
        self.states = []
        self.actions = []
        self.recording_start_time = 0
        self.recording_duration = 0
        # 等待任何正在进行的图像保存任务完成
        for task in self.image_tasks:
            task.result()
        self.image_tasks = []
    
    def save_image_async(self, img_msg, camera_name, compressed=True):
        """异步保存图像数据"""
        if camera_name not in self.img_dirs or not os.path.exists(self.img_dirs[camera_name]):
            return
        
        task = self.executor.submit(self._save_image, img_msg, camera_name, compressed)
        self.image_tasks.append(task)
        # 清理已完成的任务
        self.image_tasks = [t for t in self.image_tasks if not t.done()]
    
    def _save_image(self, img_msg, camera_name, compressed=True):
        """实际保存图像的工作函数"""
        try:
            timestamp = img_msg.header.stamp.to_sec()
            
            if compressed:
                np_arr = np.frombuffer(img_msg.data, np.uint8)
                image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            else:
                image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
                
            filename = f"{timestamp:.6f}.jpg"
            save_path = os.path.join(self.img_dirs[camera_name], filename)
            cv2.imwrite(save_path, image)
            
        except Exception as e:
            rospy.logerr(f"保存{camera_name}相机图像时出错: {str(e)}")
    
    def add_state_data(self, timestamp, arm_left, arm_right, grippers):
        """添加一帧机器人状态数据"""
        state = [timestamp]
        state.extend(arm_left)  # 左臂7个关节
        state.append(grippers[0])  # 左夹爪
        state.extend(arm_right)  # 右臂7个关节
        state.append(grippers[1])  # 右夹爪
        self.states.append(state)
        return len(self.states)
    
    def add_action_data(self, timestamp, arm_left_exp, arm_right_exp, gripper_actions):
        """添加一帧期望动作数据"""
        action = [timestamp]
        action.extend(arm_left_exp)  # 左臂期望位置
        action.append(gripper_actions[0])  # 左夹爪期望位置
        action.extend(arm_right_exp)  # 右臂期望位置
        action.append(gripper_actions[1])  # 右夹爪期望位置
        self.actions.append(action)
        return len(self.actions)
    
    def save_csv_data(self):
        """保存状态和动作数据到CSV文件"""
        if len(self.states) == 0 and len(self.actions) == 0:
            return False
            
        # 如果有数据要保存，先检查目录是否存在
        if self.save_dir is None:
            self.start_new_recording()
        
        try:
            # 保存实际状态数据
            state_csv_path = os.path.join(self.csv_dir, 'states.csv')
            with open(state_csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'timestamp',
                    'left_arm_1', 'left_arm_2', 'left_arm_3', 'left_arm_4',
                    'left_arm_5', 'left_arm_6', 'left_arm_7', 'left_gripper',
                    'right_arm_1', 'right_arm_2', 'right_arm_3', 'right_arm_4',
                    'right_arm_5', 'right_arm_6', 'right_arm_7', 'right_gripper'
                ])
                writer.writerows(self.states)
                
            # 保存期望位置(action)数据
            action_csv_path = os.path.join(self.csv_dir, 'actions.csv')
            with open(action_csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'timestamp',
                    'left_arm_exp_1', 'left_arm_exp_2', 'left_arm_exp_3', 'left_arm_exp_4',
                    'left_arm_exp_5', 'left_arm_exp_6', 'left_arm_exp_7', 'left_gripper_exp',
                    'right_arm_exp_1', 'right_arm_exp_2', 'right_arm_exp_3', 'right_arm_exp_4',
                    'right_arm_exp_5', 'right_arm_exp_6', 'right_arm_exp_7', 'right_gripper_exp'
                ])
                writer.writerows(self.actions)
                
            # 保存元数据
            self.save_metadata()
            return True
        except Exception as e:
            rospy.logerr(f"保存CSV数据时出错: {str(e)}")
            return False
    
    def save_metadata(self):
        """保存元数据，包括录制时长和数据统计信息"""
        if self.save_dir is None:
            return None
            
        try:
            metadata_path = os.path.join(self.save_dir, 'metadata.txt')
            with open(metadata_path, 'w') as f:
                f.write(f"录制开始时间: {datetime.fromtimestamp(self.recording_start_time).strftime('%Y-%m-%d %H:%M:%S')}\n")
                end_time = self.recording_start_time + self.recording_duration
                f.write(f"录制结束时间: {datetime.fromtimestamp(end_time).strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"录制总时长: {self.format_duration(self.recording_duration)}\n")
                f.write(f"状态数据帧数: {len(self.states)}\n")
                f.write(f"动作数据帧数: {len(self.actions)}\n")
                
                # 统计各相机图像数量
                image_counts = {}
                for cam_name in ['front', 'left', 'right']:
                    if cam_name in self.img_dirs and os.path.exists(self.img_dirs[cam_name]):
                        image_counts[cam_name] = len([f for f in os.listdir(self.img_dirs[cam_name]) if f.endswith('.jpg')])
                    else:
                        image_counts[cam_name] = 0
                        
                f.write(f"\n图像数据统计:\n")
                for cam_name, count in image_counts.items():
                    f.write(f"  {cam_name}相机: {count}张\n")
                
                # 各相机平均帧率
                total_duration = self.recording_duration if self.recording_duration > 0 else 1
                f.write("\n平均帧率统计:\n")
                for cam_name, count in image_counts.items():
                    avg_fps = count / total_duration
                    f.write(f"  {cam_name}相机: {avg_fps:.2f} fps\n")
                
                # 状态数据平均帧率
                if len(self.states) > 0:
                    state_fps = len(self.states) / total_duration
                    f.write(f"  状态数据: {state_fps:.2f} fps\n")
                
            return metadata_path
        except Exception as e:
            rospy.logerr(f"保存元数据时出错: {str(e)}")
            return None
    
    # def create_zip_archive(self, compression_level=6, signal=None, output_dir=None):
    #     """将录制数据打包为ZIP文件
        
    #     Args:
    #         compression_level: ZIP压缩级别(0-9)
    #         signal: 进度信号
    #         output_dir: ZIP输出目录，如果为None则使用默认目录
    #     """
    #     if self.save_dir is None or not os.path.exists(self.save_dir):
    #         return None
            
    #     try:
    #         # 创建ZIP文件名
    #         dir_name = os.path.basename(self.save_dir)
            
    #         # 确定ZIP保存路径
    #         if output_dir and os.path.exists(output_dir):
    #             zip_dir = output_dir
    #         else:
    #             zip_dir = os.path.dirname(self.save_dir)  # 默认保存在数据目录的同级目录
                
    #         zip_path = os.path.join(zip_dir, f"{dir_name}.zip")
            
    #         # 列出要添加到ZIP的所有文件
    #         files_to_zip = []
    #         for root, _, files in os.walk(self.save_dir):
    #             for file in files:
    #                 files_to_zip.append(os.path.join(root, file))
            
    #         # 设置进度信号
    #         total_files = len(files_to_zip)
    #         current_file = 0
            
    #         with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED, compresslevel=compression_level) as zipf:
    #             for file_path in files_to_zip:
    #                 # 获取相对路径
    #                 rel_path = os.path.relpath(file_path, os.path.dirname(self.save_dir))
    #                 zipf.write(file_path, rel_path)
                    
    #                 # 更新进度信号
    #                 current_file += 1
    #                 if signal:
    #                     progress = int(current_file * 100 / total_files)
    #                     signal.emit(progress)
            
    #         return zip_path
            
    #     except Exception as e:
    #         rospy.logerr(f"创建ZIP归档时出错: {str(e)}")
    #         return None
    
    def create_zip_archive(self, compression_level=6, signal=None, output_dir=None):
        """将录制数据打包为ZIP文件
        
        Args:
            compression_level: ZIP压缩级别(0-9)
            signal: 进度信号
            output_dir: ZIP输出目录，如果为None则使用默认目录
        """
        if self.save_dir is None or not os.path.exists(self.save_dir):
            return None
            
        try:
            # 创建ZIP文件名
            dir_name = os.path.basename(self.save_dir)
            
            # 确定ZIP保存路径
            if output_dir and os.path.exists(output_dir):
                zip_dir = output_dir
            else:
                zip_dir = os.path.dirname(self.save_dir)  # 默认保存在数据目录的同级目录
                
            zip_path = os.path.join(zip_dir, f"{dir_name}.zip")
            
            # 列出要添加到ZIP的所有文件
            files_to_zip = []
            for root, _, files in os.walk(self.save_dir):
                for file in files:
                    files_to_zip.append(os.path.join(root, file))
            
            # 设置进度信号
            total_files = len(files_to_zip)
            current_file = 0
            
            with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED, compresslevel=compression_level) as zipf:
                for file_path in files_to_zip:
                    # 获取相对路径
                    rel_path = os.path.relpath(file_path, os.path.dirname(self.save_dir))
                    zipf.write(file_path, rel_path)
                    
                    # 更新进度信号
                    current_file += 1
                    if signal:
                        progress = int(current_file * 100 / total_files)
                        signal.emit(progress)
            
            return zip_path
            
        except Exception as e:
            rospy.logerr(f"创建ZIP归档时出错: {str(e)}")
            return None
    # def get_session_summary(self):
    #     """获取当前会话的录制摘要"""
    #     if not os.path.exists(self.base_dir):
    #         return []
            
    #     summary = []
    #     all_items = os.listdir(self.base_dir)
        
    #     # 获取所有录制目录和ZIP文件
    #     recordings = [d for d in all_items 
    #                if os.path.isdir(os.path.join(self.base_dir, d)) and not d.startswith('.')]
    #     zip_files = [f for f in all_items
    #               if f.endswith('.zip') and os.path.isfile(os.path.join(self.base_dir, f))]
        
    #     # 处理录制目录
    #     for rec_dir in sorted(recordings):
    #         metadata_path = os.path.join(self.base_dir, rec_dir, 'metadata.txt')
    #         if os.path.exists(metadata_path):
    #             with open(metadata_path, 'r') as f:
    #                 content = f.read()
    #                 duration_match = next((line for line in content.split('\n') if "录制总时长" in line), "未知时长")
    #                 frames_match = next((line for line in content.split('\n') if "状态数据帧数" in line), "未知帧数")
    #             summary.append({
    #                 'name': rec_dir,
    #                 'type': 'directory',
    #                 'info': f"{duration_match.strip()}, {frames_match.strip()}"
    #             })
    #         else:
    #             summary.append({
    #                 'name': rec_dir,
    #                 'type': 'directory',
    #                 'info': "无元数据"
    #             })
        
    #     # 处理ZIP文件
    #     for zip_file in sorted(zip_files):
    #         zip_size = os.path.getsize(os.path.join(self.base_dir, zip_file))
    #         summary.append({
    #             'name': zip_file,
    #             'type': 'zip',
    #             'info': f"ZIP归档文件, 大小: {self.format_size(zip_size)}"
    #         })
        
    #     return summary
    
    def get_session_summary(self):
        """获取当前会话的录制摘要，同时包括主目录和ZIP目录的文件"""
        summary = []
        
        # 检查基本目录是否存在
        if os.path.exists(self.base_dir):
            # 处理基本目录中的内容
            all_items = os.listdir(self.base_dir)
            
            # 获取所有录制目录和ZIP文件
            recordings = [d for d in all_items 
                        if os.path.isdir(os.path.join(self.base_dir, d)) and not d.startswith('.')]
            zip_files = [f for f in all_items
                    if f.endswith('.zip') and os.path.isfile(os.path.join(self.base_dir, f))]
            
            # 处理录制目录
            for rec_dir in sorted(recordings):
                metadata_path = os.path.join(self.base_dir, rec_dir, 'metadata.txt')
                if os.path.exists(metadata_path):
                    with open(metadata_path, 'r') as f:
                        content = f.read()
                        duration_match = next((line for line in content.split('\n') if "录制总时长" in line), "未知时长")
                        frames_match = next((line for line in content.split('\n') if "状态数据帧数" in line), "未知帧数")
                    summary.append({
                        'name': rec_dir,
                        'type': 'directory',
                        'source_dir': self.base_dir,
                        'info': f"{duration_match.strip()}, {frames_match.strip()}"
                    })
                else:
                    summary.append({
                        'name': rec_dir,
                        'type': 'directory',
                        'source_dir': self.base_dir,
                        'info': "无元数据"
                    })
            
            # 处理基本目录中的ZIP文件
            for zip_file in sorted(zip_files):
                zip_size = os.path.getsize(os.path.join(self.base_dir, zip_file))
                
                # 检查ZIP文件是否与目录对应
                zip_base_name = zip_file[:-4] if zip_file.endswith('.zip') else zip_file
                is_match = any(rec_dir == zip_base_name for rec_dir in recordings)
                
                summary.append({
                    'name': zip_file,
                    'type': 'zip',
                    'source_dir': self.base_dir,
                    'info': f"ZIP归档文件, 大小: {self.format_size(zip_size)}",
                    'matches_dir': is_match
                })
        
        # 检查ZIP目录是否存在且与基本目录不同
        zip_dir = getattr(self.args, 'zip_dir', None) if hasattr(self, 'args') else None
        if zip_dir and os.path.exists(zip_dir) and zip_dir != self.base_dir:
            # 处理ZIP目录中的ZIP文件
            all_zip_items = os.listdir(zip_dir)
            zip_files = [f for f in all_zip_items
                    if f.endswith('.zip') and os.path.isfile(os.path.join(zip_dir, f))]
            
            for zip_file in sorted(zip_files):
                zip_size = os.path.getsize(os.path.join(zip_dir, zip_file))
                
                # 提取不带.zip后缀的文件名
                zip_base_name = zip_file[:-4] if zip_file.endswith('.zip') else zip_file
                
                # 检查是否已存在同名ZIP文件（从基本目录读取的）
                duplicate = False
                for item in summary:
                    if item['type'] == 'zip' and item['name'] == zip_file:
                        duplicate = True
                        break
                
                if not duplicate:
                    summary.append({
                        'name': zip_file,
                        'type': 'zip',
                        'source_dir': zip_dir,
                        'info': f"ZIP归档文件, 大小: {self.format_size(zip_size)}",
                        'matches_dir': False  # 我们在这里不知道是否匹配目录
                    })
        
        return summary

    @staticmethod
    def format_duration(seconds):
        """将秒数格式化为时:分:秒格式"""
        hours, remainder = divmod(seconds, 3600)
        minutes, seconds = divmod(remainder, 60)
        return f"{int(hours):02d}:{int(minutes):02d}:{int(seconds):02d}"
    
    @staticmethod
    def format_size(size_bytes):
        """将字节大小格式化为人类可读格式"""
        if size_bytes < 1024:
            return f"{size_bytes} B"
        elif size_bytes < 1024 * 1024:
            return f"{size_bytes / 1024:.2f} KB"
        elif size_bytes < 1024 * 1024 * 1024:
            return f"{size_bytes / (1024 * 1024):.2f} MB"
        else:
            return f"{size_bytes / (1024 * 1024 * 1024):.2f} GB"

class ROSRecorderGUI(QMainWindow):
    """ROS数据记录器与处理系统GUI主窗口"""
    
    def __init__(self, args):
        super().__init__()
        
        # 设置参数
        self.args = args
        # 设置默认ZIP目录
        if not hasattr(self.args, 'zip_dir') or self.args.zip_dir is None:
            self.args.zip_dir = self.args.save_dir  # 默认与save_dir相同
        
        # 初始化数据
        self.topic_monitor = TopicMonitor()
        self.data_storage = DataStorage(args.save_dir, args=args)  # 传递args参数
        self.is_recording = False
        self.is_compressing = False  # 是否在压缩中标志
        self.is_processing = False   # 是否在处理数据中标志
        self.recording_start_time = 0
        
        # 数据缓存
        self.latest_gripper_action = [0.0, 0.0]  # 最新的夹爪控制指令
        self.latest_grippers = [0.0, 0.0]  # 最新的夹爪实际位置
        
        # 设置界面
        self.setup_chinese_ui()
        self.init_ui()
        
        # 设置定时器，更新界面状态
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_status)
        self.update_timer.start(500)  # 每0.5秒更新一次界面
        
        # 初始化ROS节点
        self.init_ros_node()
        
        # 显示就绪
        self.log_message("ROS数据记录器已初始化，等待话题数据...")
    
    def setup_chinese_ui(self):
        """设置中文UI支持"""
        # 尝试设置中文字体
        chinese_fonts = ['Microsoft YaHei', 'SimHei', 'WenQuanYi Micro Hei', 'Noto Sans CJK SC', 'Noto Sans SC']
        
        app = QApplication.instance()
        font_db = QFontDatabase()
        system_fonts = font_db.families()
        
        # 查找第一个可用的中文字体
        available_font = None
        for font_name in chinese_fonts:
            if any(font_name.lower() in f.lower() for f in system_fonts):
                available_font = font_name
                break
        
        # 如果找到支持中文的字体，设置为应用默认字体
        if available_font:
            font = QFont(available_font, 9)
            app.setFont(font)
            print(f"已设置中文字体: {available_font}")
        else:
            # 无合适中文字体，使用系统默认
            print("未找到合适的中文字体，使用系统默认字体")
        
    def init_ui(self):
        """初始化用户界面"""
        # 设置窗口
        self.setWindowTitle("青龙ROS1数据记录与处理系统")
        self.setGeometry(100, 100, 1100, 700)
        
        # 创建主窗口部件
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        
        # 主布局
        main_layout = QVBoxLayout(main_widget)
        
        # 创建标题标签
        title_label = QLabel("青龙ROS1数据记录与处理系统")
        title_label.setFont(QFont('Arial', 18, QFont.Bold))
        title_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title_label)
        
        # 创建标签页
        tabs = QTabWidget()
        main_layout.addWidget(tabs)
        
        # 录制标签页
        record_tab = QWidget()
        tabs.addTab(record_tab, "实时监控与录制")
        
        # 管理标签页
        manage_tab = QWidget()
        tabs.addTab(manage_tab, "数据管理")
        
        # 数据处理标签页
        process_tab = QWidget()
        tabs.addTab(process_tab, "数据处理")
        
        # 设置标签页
        settings_tab = QWidget()
        tabs.addTab(settings_tab, "设置")
        
        # 回放标签页
        playback_tab = QWidget()
        tabs.addTab(playback_tab, "轨迹回放")

        # 布置录制标签页
        self.setup_record_tab(record_tab)
        
        # 布置管理标签页
        self.setup_manage_tab(manage_tab)
        
        # 布置数据处理标签页
        self.setup_process_tab(process_tab)
        
        # 布置设置标签页
        self.setup_settings_tab(settings_tab)
        
        # 布置回放标签页
        self.setup_playback_tab(playback_tab)

        # 状态栏
        self.statusbar = self.statusBar()
        self.statusbar.showMessage("就绪")
        
    def setup_record_tab(self, tab):
        """设置录制标签页的UI"""
        layout = QVBoxLayout(tab)
        
        # 话题监控区
        topic_group = QGroupBox("话题状态监控")
        topic_layout = QVBoxLayout()
        
        # 话题表格
        self.topic_table = QTableWidget(7, 4)  # 7行4列
        self.topic_table.setHorizontalHeaderLabels(["话题", "频率", "消息计数", "状态"])
        self.topic_table.verticalHeader().setVisible(False)
        self.topic_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        
        # 填充话题表格
        topic_names = ['front_camera', 'left_camera', 'right_camera', 
                       'puppet_state', 'gripper_left', 'gripper_right', 'gripper_action']
        for i, topic in enumerate(topic_names):
            self.topic_table.setItem(i, 0, QTableWidgetItem(topic))
            self.topic_table.setItem(i, 1, QTableWidgetItem("0.0 Hz"))
            self.topic_table.setItem(i, 2, QTableWidgetItem("0"))
            status_item = QTableWidgetItem("WAITING")
            status_item.setForeground(QColor("orange"))
            self.topic_table.setItem(i, 3, status_item)
            
        topic_layout.addWidget(self.topic_table)
        topic_group.setLayout(topic_layout)
        layout.addWidget(topic_group)
        
        # 录制状态区
        recording_group = QGroupBox("录制状态")
        recording_layout = QVBoxLayout()
        
        # 录制状态信息
        status_layout = QGridLayout()
        
        # 当前状态
        status_layout.addWidget(QLabel("当前状态:"), 0, 0)
        self.record_status_label = QLabel("未录制")
        self.record_status_label.setStyleSheet("font-weight: bold; color: gray;")
        status_layout.addWidget(self.record_status_label, 0, 1)
        
        # 录制时长
        status_layout.addWidget(QLabel("录制时长:"), 0, 2)
        self.record_duration_label = QLabel("00:00:00")
        status_layout.addWidget(self.record_duration_label, 0, 3)
        
        # 状态数据帧数
        status_layout.addWidget(QLabel("状态帧数:"), 1, 0)
        self.state_frames_label = QLabel("0")
        status_layout.addWidget(self.state_frames_label, 1, 1)
        
        # 图像总数
        status_layout.addWidget(QLabel("图像总数:"), 1, 2)
        self.image_count_label = QLabel("0")
        status_layout.addWidget(self.image_count_label, 1, 3)
        
        # 保存目录
        status_layout.addWidget(QLabel("保存目录:"), 2, 0)
        self.save_dir_label = QLabel("-")
        self.save_dir_label.setWordWrap(True)
        status_layout.addWidget(self.save_dir_label, 2, 1, 1, 3)
        
        recording_layout.addLayout(status_layout)
        
        # ZIP压缩进度条 - 移到第一页
        progress_layout = QHBoxLayout()
        progress_layout.addWidget(QLabel("ZIP压缩进度:"))
        
        self.zip_progress = QProgressBar()
        self.zip_progress.setRange(0, 100)
        self.zip_progress.setValue(0)
        self.zip_progress.setVisible(False)
        progress_layout.addWidget(self.zip_progress)
        
        recording_layout.addLayout(progress_layout)
        
        # 控制按钮区
        buttons_layout = QHBoxLayout()
        
        # 开始录制按钮
        self.start_button = QPushButton("开始录制")
        self.start_button.setStyleSheet("background-color: #4CAF50; color: white;")
        self.start_button.clicked.connect(self.start_recording)
        buttons_layout.addWidget(self.start_button)
        
        # 停止录制按钮
        self.stop_button = QPushButton("停止录制")
        self.stop_button.setStyleSheet("background-color: #F44336; color: white;")
        self.stop_button.clicked.connect(self.stop_recording)
        self.stop_button.setEnabled(False)
        buttons_layout.addWidget(self.stop_button)
        
        # 重置按钮
        self.reset_button = QPushButton("重置")
        self.reset_button.clicked.connect(self.reset_state)
        buttons_layout.addWidget(self.reset_button)
        
        recording_layout.addLayout(buttons_layout)
        recording_group.setLayout(recording_layout)
        layout.addWidget(recording_group)
        
        # 日志区
        log_group = QGroupBox("日志信息")
        log_layout = QVBoxLayout()
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        log_layout.addWidget(self.log_text)
        
        log_group.setLayout(log_layout)
        layout.addWidget(log_group)
        
    def setup_manage_tab(self, tab):
        """设置数据管理标签页的UI"""
        layout = QVBoxLayout(tab)
        
        # 录制列表区域
        recordings_group = QGroupBox("录制数据列表")
        recordings_layout = QVBoxLayout()
        
        # 录制列表表格
        self.recordings_table = QTableWidget(0, 3)  # 0行3列
        self.recordings_table.setHorizontalHeaderLabels(["名称", "类型", "信息"])
        self.recordings_table.verticalHeader().setVisible(False)
        self.recordings_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.recordings_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.recordings_table.setSelectionMode(QTableWidget.SingleSelection)
        
        recordings_layout.addWidget(self.recordings_table)
        
        # 操作按钮区
        buttons_layout = QHBoxLayout()
        
        # 刷新按钮
        refresh_button = QPushButton("刷新列表")
        refresh_button.clicked.connect(self.refresh_recordings_list)
        buttons_layout.addWidget(refresh_button)
        
        # 打开目录按钮
        open_dir_button = QPushButton("打开目录")
        open_dir_button.clicked.connect(self.open_selected_dir)
        buttons_layout.addWidget(open_dir_button)
        
        # 创建ZIP按钮
        self.create_zip_button = QPushButton("创建ZIP")
        self.create_zip_button.clicked.connect(self.create_zip_for_selected)
        buttons_layout.addWidget(self.create_zip_button)
        
        # 删除按钮
        delete_button = QPushButton("删除")
        delete_button.setStyleSheet("background-color: #F44336; color: white;")
        delete_button.clicked.connect(self.delete_selected)
        buttons_layout.addWidget(delete_button)
        
        recordings_layout.addLayout(buttons_layout)
        
        recordings_group.setLayout(recordings_layout)
        layout.addWidget(recordings_group)
        
        # 初始加载录制列表
        self.refresh_recordings_list()
        
    def setup_process_tab(self, tab):
        """设置数据处理标签页的UI"""
        layout = QVBoxLayout(tab)
        
        # 数据处理区域
        process_group = QGroupBox("数据处理")
        process_layout = QVBoxLayout()
        
        # 源数据目录选择
        source_layout = QHBoxLayout()
        source_layout.addWidget(QLabel("源数据目录:"))
        self.source_dir_edit = QLabel(self.args.save_dir)
        source_layout.addWidget(self.source_dir_edit)
        
        source_browse_button = QPushButton("浏览...")
        source_browse_button.clicked.connect(self.browse_source_dir)
        source_layout.addWidget(source_browse_button)
        
        process_layout.addLayout(source_layout)
        
        # 输出目录选择
        output_layout = QHBoxLayout()
        output_layout.addWidget(QLabel("输出目录:"))
        self.output_dir_edit = QLabel(os.path.join(self.args.save_dir, "processed"))
        output_layout.addWidget(self.output_dir_edit)
        
        output_browse_button = QPushButton("浏览...")
        output_browse_button.clicked.connect(self.browse_output_dir)
        output_layout.addWidget(output_browse_button)
        
        process_layout.addLayout(output_layout)
        
        # 处理选项
        options_layout = QGridLayout()
        
        # 处理线程数
        options_layout.addWidget(QLabel("处理线程数:"), 0, 0)
        self.process_threads_combo = QComboBox()
        for i in range(1, 9):
            self.process_threads_combo.addItem(f"{i}")
        self.process_threads_combo.setCurrentIndex(2)  # 默认3线程
        options_layout.addWidget(self.process_threads_combo, 0, 1)
        
        # 自动删除失败结果选项
        options_layout.addWidget(QLabel("自动删除失败结果:"), 1, 0)
        self.delete_failed_check = QCheckBox()
        self.delete_failed_check.setChecked(True)  # 默认删除失败的处理
        options_layout.addWidget(self.delete_failed_check, 1, 1)
        
        process_layout.addLayout(options_layout)
        
        # 处理按钮
        start_process_button = QPushButton("开始处理")
        start_process_button.setStyleSheet("background-color: #4CAF50; color: white;")
        start_process_button.clicked.connect(self.start_data_processing)
        process_layout.addWidget(start_process_button)
        
        # 处理进度
        self.process_log = QTextEdit()
        self.process_log.setReadOnly(True)
        self.process_log.setMinimumHeight(300)
        process_layout.addWidget(self.process_log)
        
        process_group.setLayout(process_layout)
        layout.addWidget(process_group)
        
    def setup_settings_tab(self, tab):
        """设置配置标签页的UI"""
        layout = QVBoxLayout(tab)
        
        # 存储设置区
        storage_group = QGroupBox("存储设置")
        storage_layout = QGridLayout()
        
        # 保存目录
        storage_layout.addWidget(QLabel("保存目录:"), 0, 0)
        self.save_dir_edit = QLabel(self.args.save_dir)
        storage_layout.addWidget(self.save_dir_edit, 0, 1)
        
        browse_button = QPushButton("浏览...")
        browse_button.clicked.connect(self.browse_save_dir)
        storage_layout.addWidget(browse_button, 0, 2)
        
        # ZIP保存目录
        storage_layout.addWidget(QLabel("ZIP保存目录:"), 1, 0)
        self.zip_dir_edit = QLabel(self.args.zip_dir)
        storage_layout.addWidget(self.zip_dir_edit, 1, 1)
        
        zip_browse_button = QPushButton("浏览...")
        zip_browse_button.clicked.connect(self.browse_zip_dir)
        storage_layout.addWidget(zip_browse_button, 1, 2)
        
        # ZIP设置
        storage_layout.addWidget(QLabel("自动创建ZIP:"), 2, 0)
        self.auto_zip_check = QCheckBox()
        self.auto_zip_check.setChecked(self.args.auto_zip)
        storage_layout.addWidget(self.auto_zip_check, 2, 1)
        
        storage_layout.addWidget(QLabel("ZIP后删除原始文件:"), 3, 0)
        self.delete_after_zip_check = QCheckBox()
        self.delete_after_zip_check.setChecked(self.args.delete_after_zip)
        storage_layout.addWidget(self.delete_after_zip_check, 3, 1)
        
        storage_layout.addWidget(QLabel("ZIP压缩级别:"), 4, 0)
        self.zip_level_combo = QComboBox()
        for i in range(10):
            self.zip_level_combo.addItem(f"{i} - {'无压缩' if i==0 else '最大压缩' if i==9 else '标准压缩'}")
        self.zip_level_combo.setCurrentIndex(self.args.zip_compression)
        storage_layout.addWidget(self.zip_level_combo, 4, 1)
        
        storage_group.setLayout(storage_layout)
        layout.addWidget(storage_group)
        
        # 话题设置区
        topic_group = QGroupBox("话题设置")
        topic_layout = QGridLayout()
        
        # 是否使用压缩图像
        topic_layout.addWidget(QLabel("使用压缩图像:"), 0, 0)
        self.compress_check = QCheckBox()
        self.compress_check.setChecked(self.args.compress)
        topic_layout.addWidget(self.compress_check, 0, 1)
        
        # 忽略缺失话题
        topic_layout.addWidget(QLabel("忽略缺失的话题:"), 1, 0)
        self.ignore_missing_topics_check = QCheckBox()
        self.ignore_missing_topics_check.setChecked(self.args.ignore_missing_topics)
        topic_layout.addWidget(self.ignore_missing_topics_check, 1, 1)
        
        topic_group.setLayout(topic_layout)
        layout.addWidget(topic_group)
        
        # 保存设置按钮
        save_button = QPushButton("保存设置")
        save_button.clicked.connect(self.save_settings)
        layout.addWidget(save_button)
        
        # 添加弹性空间
        layout.addStretch()

    def setup_playback_tab(self, tab):
        """设置回放标签页的UI"""
        layout = QVBoxLayout(tab)
        
        # 轨迹文件选择区
        file_group = QGroupBox("轨迹文件选择")
        file_layout = QVBoxLayout(file_group)
        
        # 当前目录的处理后结果
        results_layout = QHBoxLayout()
        results_layout.addWidget(QLabel("当前处理结果目录:"))
        
        self.playback_dir_combo = QComboBox()
        results_layout.addWidget(self.playback_dir_combo)
        
        refresh_button = QPushButton("刷新")
        refresh_button.clicked.connect(self.refresh_playback_dirs)
        results_layout.addWidget(refresh_button)
        
        file_layout.addLayout(results_layout)
        
        # 手动选择HDF5文件
        hdf5_layout = QHBoxLayout()
        hdf5_layout.addWidget(QLabel("或直接选择HDF5文件:"))
        
        self.hdf5_path_edit = QLineEdit()
        self.hdf5_path_edit.setReadOnly(True)
        hdf5_layout.addWidget(self.hdf5_path_edit)
        
        hdf5_browse_button = QPushButton("浏览...")
        hdf5_browse_button.clicked.connect(self.browse_hdf5_file)
        hdf5_layout.addWidget(hdf5_browse_button)
        
        file_layout.addLayout(hdf5_layout)
        
        layout.addWidget(file_group)
        
        # 启动回放按钮
        start_playback_button = QPushButton("启动轨迹回放")
        start_playback_button.setStyleSheet("background-color: #4CAF50; color: white; font-size: 14px; padding: 10px;")
        start_playback_button.clicked.connect(self.start_trajectory_playback)
        layout.addWidget(start_playback_button)
        
        # 说明文本
        info_text = QTextEdit()
        info_text.setReadOnly(True)
        info_text.setPlainText("""
    使用说明:

    1. 轨迹回放需要已经处理后的数据，包含HDF5文件和同步后的图像。
    2. 您可以选择下拉列表中的处理结果目录，或直接浏览选择HDF5文件。
    3. 点击"启动轨迹回放"按钮打开回放控制窗口。
    4. 在回放控制窗口中，您可以:
    - 控制回放的暂停和继续
    - 调整回放速率
    - 选择是否显示相机图像
        
    注意事项:
    - 回放过程中会发布机器人控制指令到ROS话题，确保安全。
    - 轨迹回放时请保持对机器人的监控，必要时随时停止。
    - 如果显示图像，请确保数据目录中包含camera文件夹及对应帧图像。
        """)
        layout.addWidget(info_text)
        
        # 所有UI元素创建完成后再刷新目录列表
        self.refresh_playback_dirs()
    def refresh_playback_dirs(self):
        """刷新可用于回放的处理结果目录"""
        self.playback_dir_combo.clear()
        
        # 检查处理输出目录
        output_dir = os.path.join(self.args.save_dir, "processed", "final_output")
        if os.path.exists(output_dir):
            result_dirs = [d for d in os.listdir(output_dir) 
                        if os.path.isdir(os.path.join(output_dir, d))]
            
            for d in sorted(result_dirs, reverse=True):  # 最新的排在前面
                h5_path = os.path.join(output_dir, d, "record", "aligned_joints.h5")
                if os.path.exists(h5_path):
                    self.playback_dir_combo.addItem(d, h5_path)
        
        # 如果有可用目录，自动选择第一个
        if self.playback_dir_combo.count() > 0:
            self.playback_dir_combo.setCurrentIndex(0)
            self.hdf5_path_edit.setText(self.playback_dir_combo.currentData())

    def browse_hdf5_file(self):
        """浏览并选择HDF5文件"""
        file_path, _ = QFileDialog.getOpenFileName(
            self, "选择HDF5文件", self.args.save_dir, "HDF5文件 (*.h5 *.hdf5);;所有文件 (*.*)"
        )
        
        if file_path:
            self.hdf5_path_edit.setText(file_path)
            # 当手动选择文件时，取消下拉选择
            self.playback_dir_combo.setCurrentIndex(-1)

    def start_trajectory_playback(self):
        """启动轨迹回放对话框"""
        hdf5_path = ""
        
        # 优先使用手动选择的文件
        if self.hdf5_path_edit.text() and os.path.exists(self.hdf5_path_edit.text()):
            hdf5_path = self.hdf5_path_edit.text()
        # 其次使用下拉列表选择的目录
        elif self.playback_dir_combo.currentData():
            hdf5_path = self.playback_dir_combo.currentData()
        
        if not hdf5_path or not os.path.exists(hdf5_path):
            QMessageBox.warning(self, "错误", "请先选择有效的HDF5文件")
            return
        
        # 创建并显示轨迹回放对话框
        player_dialog = TrajectoryPlayerDialog(hdf5_path, self)
        player_dialog.exec_()

    # 新增方法：浏览ZIP保存目录
    def browse_zip_dir(self):
        """浏览并选择ZIP保存目录"""
        # 如果在录制或压缩中，不允许更改目录
        if self.is_recording or self.is_compressing:
            self.log_message("正在录制或压缩中，无法更改ZIP保存目录", error=True)
            return
            
        dir_path = QFileDialog.getExistingDirectory(
            self, "选择ZIP保存目录", self.args.zip_dir,
            QFileDialog.ShowDirsOnly
        )
        
        if dir_path:
            self.zip_dir_edit.setText(dir_path)
            self.args.zip_dir = dir_path
            self.log_message(f"ZIP保存目录已更改为: {dir_path}")
    def init_ros_node(self):
        """初始化ROS节点和订阅器"""
        try:
            rospy.init_node('data_recorder_gui', anonymous=True)
            
            # 设置图像订阅器
            rospy.Subscriber(
                self.args.img_front_topic,
                CompressedImage if self.args.compress else Image,
                self.img_front_callback,
                queue_size=1000,
                tcp_nodelay=True
            )
            rospy.Subscriber(
                self.args.img_left_topic,
                CompressedImage if self.args.compress else Image,
                self.img_left_callback,
                queue_size=1000,
                tcp_nodelay=True
            )
            rospy.Subscriber(
                self.args.img_right_topic,
                CompressedImage if self.args.compress else Image,
                self.img_right_callback,
                queue_size=1000,
                tcp_nodelay=True
            )
            
            # 设置其他数据订阅器
            rospy.Subscriber(
                self.args.puppet_state_topic,
                PuppetState,
                self.puppet_state_callback,
                queue_size=1000,
                tcp_nodelay=True
            )
            rospy.Subscriber(
                self.args.gripper_left_topic,
                JointState,
                self.gripper_left_callback,
                queue_size=1000,
                tcp_nodelay=True
            )
            rospy.Subscriber(
                self.args.gripper_right_topic,
                JointState,
                self.gripper_right_callback,
                queue_size=1000,
                tcp_nodelay=True
            )
            rospy.Subscriber(
                self.args.gripper_action_topic,
                JointState,
                self.gripper_action_callback,
                queue_size=1000,
                tcp_nodelay=True
            )
            
            self.log_message("ROS节点初始化成功")
        except Exception as e:
            self.log_message(f"ROS节点初始化失败: {str(e)}", error=True)
    
    # 回调函数
    def img_front_callback(self, msg):
        """前方相机图像回调"""
        self.topic_monitor.update_topic('front_camera')
        if self.is_recording:
            self.data_storage.save_image_async(msg, 'front', self.args.compress)

    def img_left_callback(self, msg):
        """左侧相机图像回调"""
        self.topic_monitor.update_topic('left_camera')
        if self.is_recording:
            self.data_storage.save_image_async(msg, 'left', self.args.compress)

    def img_right_callback(self, msg):
        """右侧相机图像回调"""
        self.topic_monitor.update_topic('right_camera')
        if self.is_recording:
            self.data_storage.save_image_async(msg, 'right', self.args.compress)

    def puppet_state_callback(self, msg):
        """机器人状态回调"""
        self.topic_monitor.update_topic('puppet_state')
        if self.is_recording:
            timestamp = msg.header.stamp.to_sec()
            
            # 添加状态数据
            self.data_storage.add_state_data(
                timestamp, 
                msg.arm_left.position, 
                msg.arm_right.position,
                self.latest_grippers
            )
            
            # 添加动作数据
            self.data_storage.add_action_data(
                timestamp,
                msg.arm_left_exp.position,
                msg.arm_right_exp.position,
                self.latest_gripper_action
            )

    def gripper_left_callback(self, msg):
        """左夹爪回调"""
        self.topic_monitor.update_topic('gripper_left')
        if len(msg.position) > 0:
            self.latest_grippers[0] = msg.position[0]

    def gripper_right_callback(self, msg):
        """右夹爪回调"""
        self.topic_monitor.update_topic('gripper_right')
        if len(msg.position) > 0:
            self.latest_grippers[1] = msg.position[0]

    def gripper_action_callback(self, msg):
        """夹爪控制指令回调"""
        self.topic_monitor.update_topic('gripper_action')
        if len(msg.position) > 0:
            self.latest_gripper_action = list(msg.position)
    
    def update_status(self):
        """更新界面状态"""
        # 更新话题状态表格
        for i, (topic_name, topic) in enumerate(self.topic_monitor.topics.items()):
            # 更新频率
            self.topic_table.item(i, 1).setText(f"{topic.freq:.1f} Hz")
            # 更新消息计数
            self.topic_table.item(i, 2).setText(f"{topic.count}")
            # 更新状态
            status = topic.get_status(self.topic_monitor.timeout)
            status_item = self.topic_table.item(i, 3)
            status_item.setText(status)
            
            if status == 'WAITING':
                status_item.setForeground(QColor("orange"))
            elif status == 'TIMEOUT':
                status_item.setForeground(QColor("red"))
            else:
                status_item.setForeground(QColor("green"))
        
        # 更新录制状态信息
        if self.is_recording:
            # 计算当前录制时长
            current_duration = time.time() - self.recording_start_time
            self.data_storage.recording_duration = current_duration
            
            # 更新录制状态标签
            self.record_status_label.setText("正在录制")
            self.record_status_label.setStyleSheet("font-weight: bold; color: green;")
            
            # 更新时长
            duration_str = self.data_storage.format_duration(current_duration)
            self.record_duration_label.setText(duration_str)
            
            # 更新帧数
            self.state_frames_label.setText(f"{len(self.data_storage.states)}")
            
            # 计算图像总数
            image_count = sum(topic.count for name, topic in self.topic_monitor.topics.items() 
                            if 'camera' in name)
            self.image_count_label.setText(f"{image_count}")
            
        else:
            # 未在录制状态
            if self.is_compressing:
                self.record_status_label.setText("ZIP压缩中")
                self.record_status_label.setStyleSheet("font-weight: bold; color: blue;")
            else:
                self.record_status_label.setText("未录制")
                self.record_status_label.setStyleSheet("font-weight: bold; color: gray;")
        
        # 更新按钮状态
        self.update_button_states()
    
    def update_button_states(self):
        """更新按钮状态"""
        # 如果在压缩中，禁用开始录制
        if self.is_compressing or self.is_processing:
            self.start_button.setEnabled(False)
            # 在管理页面也禁用ZIP按钮
            self.create_zip_button.setEnabled(False)
        else:
            # 不在压缩中，根据是否在录制决定
            self.start_button.setEnabled(not self.is_recording)
            self.create_zip_button.setEnabled(True)
            
        # 停止按钮仅在录制时启用
        self.stop_button.setEnabled(self.is_recording)
    
    def log_message(self, message, error=False):
        """添加日志消息"""
        timestamp = datetime.now().strftime('%H:%M:%S')
        if error:
            log_entry = f"<span style='color:red'>[{timestamp}] ERROR: {message}</span>"
        else:
            log_entry = f"[{timestamp}] {message}"
        
        self.log_text.append(log_entry)
        # 滚动到底部
        self.log_text.verticalScrollBar().setValue(self.log_text.verticalScrollBar().maximum())
        
        # 同时在控制台输出
        print(f"[{timestamp}] {'ERROR: ' if error else ''}{message}")

    def start_recording(self):
        """开始录制数据"""
        if self.is_recording:
            self.log_message("已在录制中，请先停止当前录制")
            return
            
        # 检查是否在压缩中
        if self.is_compressing:
            self.log_message("ZIP压缩进行中，请等待压缩完成后再开始录制", error=True)
            return
            
        # 检查所有话题状态
        if not self.topic_monitor.all_topics_active():
            # 如果设置了忽略缺失话题选项，则检查核心话题
            if self.args.ignore_missing_topics:
                # 显示警告但继续录制
                self.log_message("警告：部分非核心话题未就绪，但仍将继续录制", error=True)
            else:
                self.log_message("警告：有话题未就绪，请等待所有话题变为活跃状态", error=True)
                return
            
        # 重置状态
        self.topic_monitor.reset_counts()
        
        # 创建新的录制
        save_dir = self.data_storage.start_new_recording()
        self.is_recording = True
        self.recording_start_time = time.time()
        
        # 更新UI
        self.save_dir_label.setText(save_dir)
        self.update_button_states()
        
        # 记录开始信息
        self.log_message(f"开始新录制，保存到: {save_dir}")
    
    def stop_recording(self):
        """停止录制数据"""
        if not self.is_recording:
            return
            
        self.is_recording = False
        self.log_message("结束录制...")
        
        # 更新录制时长
        self.data_storage.recording_duration = time.time() - self.recording_start_time
        
        # 保存数据
        if len(self.data_storage.states) > 0 or len(self.data_storage.actions) > 0:
            self.data_storage.save_csv_data()
            duration_str = self.data_storage.format_duration(self.data_storage.recording_duration)
            frames = len(self.data_storage.states)
            
            self.log_message(f"数据已保存到: {self.data_storage.save_dir}")
            self.log_message(f"录制总时长: {duration_str}, 总帧数: {frames}")
            
            # 创建ZIP归档
            if self.auto_zip_check.isChecked():
                # 设置压缩状态
                self.is_compressing = True
                self.update_button_states()
                
                # 显示进度条
                self.zip_progress.setValue(0)
                self.zip_progress.setVisible(True)
                
                # 启动异步压缩任务
                self.zip_worker = ZipWorker(
                    self.data_storage,
                    self.data_storage.save_dir,
                    compression_level=self.zip_level_combo.currentIndex(),
                    delete_after=self.delete_after_zip_check.isChecked(),
                    output_dir=self.args.zip_dir  # 传递ZIP输出目录
                )
                self.zip_worker.progress_signal.connect(self.update_zip_progress)
                self.zip_worker.finished_signal.connect(self.on_zip_finished)
                self.zip_worker.error_signal.connect(self.on_zip_error)
                self.zip_worker.start()
                
                self.log_message("正在创建ZIP归档，请稍候...")
        
        # 更新UI
        self.save_dir_label.setText("-")
        self.update_button_states()
        
        # 如果没有自动创建ZIP，则立即重置存储状态
        if not self.auto_zip_check.isChecked():
            # 重置存储状态
            self.data_storage.reset()
            
            # 刷新录制列表
            self.refresh_recordings_list()
    
    def reset_state(self):
        """重置录制状态"""
        if self.is_recording:
            self.stop_recording()
        
        # 如果在压缩中，不允许重置
        if self.is_compressing:
            self.log_message("ZIP压缩进行中，请等待压缩完成后再重置", error=True)
            return
            
        # 重置所有状态
        self.data_storage.reset()
        self.log_message("状态已重置，准备新的录制...")

    # def refresh_recordings_list(self):
    #     """刷新录制列表"""
    #     # 获取录制摘要
    #     summary = self.data_storage.get_session_summary()
        
    #     # 清空表格
    #     self.recordings_table.setRowCount(0)
        
    #     # 添加数据
    #     for idx, item in enumerate(summary):
    #         self.recordings_table.insertRow(idx)
            
    #         # 设置名称
    #         name_item = QTableWidgetItem(item['name'])
    #         self.recordings_table.setItem(idx, 0, name_item)
            
    #         # 设置类型
    #         type_item = QTableWidgetItem("目录" if item['type'] == 'directory' else "ZIP文件")
    #         self.recordings_table.setItem(idx, 1, type_item)
            
    #         # 设置信息
    #         info_item = QTableWidgetItem(item['info'])
    #         self.recordings_table.setItem(idx, 2, info_item)
        
    #     self.log_message(f"已刷新录制列表，共 {len(summary)} 项")
    
    
    def refresh_recordings_list(self):
        """刷新录制列表，将ZIP文件与对应目录进行关联显示"""
        # 获取录制摘要
        summary = self.data_storage.get_session_summary()
        
        # 清空表格
        self.recordings_table.setRowCount(0)
        
        # 创建一个字典，将ZIP文件与目录进行匹配
        # 格式: {目录名: {zip文件名, zip信息, source_dir}}
        zip_mapping = {}
        zip_items = []
        dir_items = []
        
        # 分离目录和ZIP文件，建立映射关系
        for item in summary:
            name = item['name']
            if item['type'] == 'zip':
                # 去掉.zip后缀获取原始目录名
                base_name = name[:-4] if name.endswith('.zip') else name
                zip_mapping[base_name] = {
                    'name': name, 
                    'info': item['info'],
                    'source_dir': item['source_dir']
                }
                zip_items.append(item)
            else:
                dir_items.append(item)
        
        # 先添加所有目录条目，如果有对应的ZIP文件则在下方添加
        row_idx = 0
        for dir_item in sorted(dir_items, key=lambda x: x['name']):
            # 添加目录行
            self.recordings_table.insertRow(row_idx)
            dir_name = dir_item['name']
            
            # 设置目录名称
            name_item = QTableWidgetItem(dir_name)
            self.recordings_table.setItem(row_idx, 0, name_item)
            
            # 设置类型
            type_item = QTableWidgetItem("目录")
            self.recordings_table.setItem(row_idx, 1, type_item)
            
            # 设置信息
            info_item = QTableWidgetItem(dir_item['info'])
            self.recordings_table.setItem(row_idx, 2, info_item)
            
            row_idx += 1
            
            # 检查是否有对应的ZIP文件
            if dir_name in zip_mapping:
                # 添加ZIP文件行，缩进显示
                self.recordings_table.insertRow(row_idx)
                
                # 设置ZIP名称（带缩进）
                zip_name_item = QTableWidgetItem("  └─ " + zip_mapping[dir_name]['name'])
                zip_name_item.setForeground(QColor(0, 120, 215))  # 蓝色
                
                # 在表格项目的userData属性中保存source_dir信息
                zip_name_item.setData(Qt.UserRole, zip_mapping[dir_name]['source_dir'])
                
                self.recordings_table.setItem(row_idx, 0, zip_name_item)
                
                # 设置类型
                zip_type_item = QTableWidgetItem("ZIP文件")
                self.recordings_table.setItem(row_idx, 1, zip_type_item)
                
                # 设置信息
                zip_info_item = QTableWidgetItem(zip_mapping[dir_name]['info'])
                self.recordings_table.setItem(row_idx, 2, zip_info_item)
                
                row_idx += 1
                
                # 从映射中移除已处理的ZIP
                del zip_mapping[dir_name]
        
        # 添加没有对应目录的独立ZIP文件
        for zip_name, zip_info in zip_mapping.items():
            self.recordings_table.insertRow(row_idx)
            
            # 设置名称
            name_item = QTableWidgetItem(zip_info['name'])
            name_item.setForeground(QColor(0, 120, 215))  # 蓝色
            
            # 在表格项目的userData属性中保存source_dir信息
            name_item.setData(Qt.UserRole, zip_info['source_dir'])
            
            self.recordings_table.setItem(row_idx, 0, name_item)
            
            # 设置类型
            type_item = QTableWidgetItem("ZIP文件")
            self.recordings_table.setItem(row_idx, 1, type_item)
            
            # 设置信息
            info_item = QTableWidgetItem(zip_info['info'])
            self.recordings_table.setItem(row_idx, 2, info_item)
            
            row_idx += 1
        
        self.log_message(f"已刷新录制列表，共 {row_idx} 项")


    # def open_selected_dir(self):
    #     """打开选中的目录"""
    #     selected_rows = self.recordings_table.selectionModel().selectedRows()
    #     if not selected_rows:
    #         self.log_message("请先选择一项", error=True)
    #         return
            
    #     row = selected_rows[0].row()
    #     item_name = self.recordings_table.item(row, 0).text()
    #     item_type = self.recordings_table.item(row, 1).text()
        
    #     # 构建完整路径
    #     if "目录" in item_type:
    #         path = os.path.join(self.data_storage.base_dir, item_name)
    #     else:
    #         path = os.path.join(self.data_storage.base_dir, item_name)
        
    #     # 检查路径是否存在
    #     if not os.path.exists(path):
    #         self.log_message(f"路径不存在: {path}", error=True)
    #         return
            
    #     # 使用系统文件管理器打开
    #     try:
    #         if sys.platform == 'win32':
    #             os.startfile(path)
    #         elif sys.platform == 'darwin':  # macOS
    #             os.system(f'open "{path}"')
    #         else:  # Linux
    #             os.system(f'xdg-open "{path}"')
            
    #         self.log_message(f"已打开: {path}")
    #     except Exception as e:
    #         self.log_message(f"打开路径时出错: {str(e)}", error=True)
    
    def open_selected_dir(self):
        """打开选中的目录或ZIP文件"""
        selected_rows = self.recordings_table.selectionModel().selectedRows()
        if not selected_rows:
            self.log_message("请先选择一项", error=True)
            return
            
        row = selected_rows[0].row()
        item = self.recordings_table.item(row, 0)
        item_name = item.text().strip()
        item_type = self.recordings_table.item(row, 1).text()
        
        # 获取文件的源目录（如果有）
        source_dir = item.data(Qt.UserRole)
        if source_dir is None:
            source_dir = self.data_storage.base_dir
        
        # 去掉缩进字符如果有的话
        if "└─" in item_name:
            item_name = item_name.split("└─")[1].strip()
        
        # 构建完整路径
        path = os.path.join(source_dir, item_name)
        
        # 检查路径是否存在
        if not os.path.exists(path):
            self.log_message(f"路径不存在: {path}", error=True)
            return
            
        # 使用系统文件管理器打开
        try:
            if sys.platform == 'win32':
                os.startfile(path)
            elif sys.platform == 'darwin':  # macOS
                os.system(f'open "{path}"')
            else:  # Linux
                os.system(f'xdg-open "{path}"')
            
            self.log_message(f"已打开: {path}")
        except Exception as e:
            self.log_message(f"打开路径时出错: {str(e)}", error=True)


    def create_zip_for_selected(self):
        """为选中的目录创建ZIP归档"""
        # 检查是否已经在压缩或录制中
        if self.is_compressing:
            self.log_message("另一个ZIP压缩任务正在进行中，请等待完成", error=True)
            return
            
        if self.is_recording:
            self.log_message("正在录制数据，无法同时进行ZIP压缩", error=True)
            return
            
        selected_rows = self.recordings_table.selectionModel().selectedRows()
        if not selected_rows:
            self.log_message("请先选择一项", error=True)
            return
            
        row = selected_rows[0].row()
        item_name = self.recordings_table.item(row, 0).text()
        item_type = self.recordings_table.item(row, 1).text()
        
        # 只能为目录创建ZIP
        if "ZIP" in item_type:
            self.log_message("选择的项目已经是ZIP文件", error=True)
            return
            
        # 构建完整路径
        dir_path = os.path.join(self.data_storage.base_dir, item_name)
        
        # 检查路径是否存在
        if not os.path.exists(dir_path) or not os.path.isdir(dir_path):
            self.log_message(f"目录不存在: {dir_path}", error=True)
            return
            
        # 设置压缩状态
        self.is_compressing = True
        self.update_button_states()
            
        # 显示进度条
        self.zip_progress.setValue(0)
        self.zip_progress.setVisible(True)
        
        # 启动异步压缩任务
        self.zip_worker = ZipWorker(
            self.data_storage,
            dir_path,
            compression_level=self.zip_level_combo.currentIndex(),
            delete_after=self.delete_after_zip_check.isChecked(),
            output_dir=self.args.zip_dir  # 传递ZIP输出目录
        )
        self.zip_worker.progress_signal.connect(self.update_zip_progress)
        self.zip_worker.finished_signal.connect(self.on_zip_finished)
        self.zip_worker.error_signal.connect(self.on_zip_error)
        self.zip_worker.start()
        
        self.log_message(f"正在为 {item_name} 创建ZIP归档...")
    
    # def delete_selected(self):
    #     """删除选中的项目"""
    #     # 检查是否在压缩中
    #     if self.is_compressing:
    #         self.log_message("ZIP压缩进行中，请等待完成后再执行删除操作", error=True)
    #         return
            
    #     selected_rows = self.recordings_table.selectionModel().selectedRows()
    #     if not selected_rows:
    #         self.log_message("请先选择一项", error=True)
    #         return
            
    #     row = selected_rows[0].row()
    #     item_name = self.recordings_table.item(row, 0).text()
    #     item_type = self.recordings_table.item(row, 1).text()
        
    #     # 构建完整路径
    #     path = os.path.join(self.data_storage.base_dir, item_name)
        
    #     # 检查路径是否存在
    #     if not os.path.exists(path):
    #         self.log_message(f"路径不存在: {path}", error=True)
    #         return
            
    #     # 确认删除
    #     reply = QMessageBox.question(self, '确认删除', 
    #         f"确定要删除 {item_name} 吗？此操作不可撤销。", 
    #         QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            
    #     if reply == QMessageBox.No:
    #         return
            
    #     # 执行删除
    #     try:
    #         if os.path.isdir(path):
    #             shutil.rmtree(path)
    #             self.log_message(f"已删除目录: {item_name}")
    #         else:
    #             os.remove(path)
    #             self.log_message(f"已删除文件: {item_name}")
                
    #         # 刷新列表
    #         self.refresh_recordings_list()
    #     except Exception as e:
    #         self.log_message(f"删除时出错: {str(e)}", error=True)
    
    # def delete_selected(self):
    #     """删除选中的项目"""
    #     # 检查是否在压缩中
    #     if self.is_compressing:
    #         self.log_message("ZIP压缩进行中，请等待完成后再执行删除操作", error=True)
    #         return
            
    #     selected_rows = self.recordings_table.selectionModel().selectedRows()
    #     if not selected_rows:
    #         self.log_message("请先选择一项", error=True)
    #         return
            
    #     row = selected_rows[0].row()
    #     item_name = self.recordings_table.item(row, 0).text().strip()
    #     item_type = self.recordings_table.item(row, 1).text()
        
    #     # 去掉缩进字符如果有的话
    #     if "└─" in item_name:
    #         item_name = item_name.split("└─")[1].strip()
        
    #     # 构建完整路径
    #     path = os.path.join(self.data_storage.base_dir, item_name)
        
    #     # 检查路径是否存在
    #     if not os.path.exists(path):
    #         self.log_message(f"路径不存在: {path}", error=True)
    #         return
            
    #     # 确认删除
    #     reply = QMessageBox.question(self, '确认删除', 
    #         f"确定要删除 {item_name} 吗？此操作不可撤销。", 
    #         QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            
    #     if reply == QMessageBox.No:
    #         return
            
    #     # 执行删除
    #     try:
    #         if os.path.isdir(path):
    #             shutil.rmtree(path)
    #             self.log_message(f"已删除目录: {item_name}")
    #         else:
    #             os.remove(path)
    #             self.log_message(f"已删除文件: {item_name}")
                
    #         # 刷新列表
    #         self.refresh_recordings_list()
    #     except Exception as e:
    #         self.log_message(f"删除时出错: {str(e)}", error=True)

    def delete_selected(self):
        """删除选中的项目"""
        # 检查是否在压缩中
        if self.is_compressing:
            self.log_message("ZIP压缩进行中，请等待完成后再执行删除操作", error=True)
            return
            
        selected_rows = self.recordings_table.selectionModel().selectedRows()
        if not selected_rows:
            self.log_message("请先选择一项", error=True)
            return
            
        row = selected_rows[0].row()
        item = self.recordings_table.item(row, 0)
        item_name = item.text().strip()
        item_type = self.recordings_table.item(row, 1).text()
        
        # 获取文件的源目录（如果有）
        source_dir = item.data(Qt.UserRole)
        if source_dir is None:
            source_dir = self.data_storage.base_dir
        
        # 去掉缩进字符如果有的话
        if "└─" in item_name:
            item_name = item_name.split("└─")[1].strip()
        
        # 构建完整路径
        path = os.path.join(source_dir, item_name)
        
        # 检查路径是否存在
        if not os.path.exists(path):
            self.log_message(f"路径不存在: {path}", error=True)
            return
            
        # 确认删除
        reply = QMessageBox.question(self, '确认删除', 
            f"确定要删除 {item_name} 吗？此操作不可撤销。", 
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            
        if reply == QMessageBox.No:
            return
            
        # 执行删除
        try:
            if os.path.isdir(path):
                shutil.rmtree(path)
                self.log_message(f"已删除目录: {item_name}")
            else:
                os.remove(path)
                self.log_message(f"已删除文件: {item_name}")
                
            # 刷新列表
            self.refresh_recordings_list()
        except Exception as e:
            self.log_message(f"删除时出错: {str(e)}", error=True)

    def update_zip_progress(self, value):
        """更新ZIP压缩进度"""
        self.zip_progress.setValue(value)
    
    def on_zip_finished(self, zip_path):
        """ZIP压缩完成回调"""
        self.zip_progress.setValue(100)
        self.log_message(f"ZIP归档创建成功: {os.path.basename(zip_path)}")
        
        # 重置压缩状态
        self.is_compressing = False
        self.update_button_states()
        
        # 重置数据存储状态
        self.data_storage.reset()
        
        # 3秒后隐藏进度条
        QTimer.singleShot(3000, lambda: self.zip_progress.setVisible(False))
        
        # 刷新列表
        self.refresh_recordings_list()
    
    def on_zip_error(self, error_msg):
        """ZIP压缩错误回调"""
        self.zip_progress.setVisible(False)
        self.log_message(f"ZIP归档创建失败: {error_msg}", error=True)
        
        # 重置压缩状态
        self.is_compressing = False
        self.update_button_states()
        
        # 重置数据存储状态
        self.data_storage.reset()

    def browse_save_dir(self):
        """浏览并选择保存目录"""
        # 如果在录制或压缩中，不允许更改目录
        if self.is_recording or self.is_compressing:
            self.log_message("正在录制或压缩中，无法更改保存目录", error=True)
            return
            
        dir_path = QFileDialog.getExistingDirectory(
            self, "选择保存目录", self.args.save_dir,
            QFileDialog.ShowDirsOnly
        )
        
        if dir_path:
            self.save_dir_edit.setText(dir_path)
            self.args.save_dir = dir_path
            self.data_storage.base_dir = dir_path
            self.log_message(f"保存目录已更改为: {dir_path}")
            
            # 刷新录制列表
            self.refresh_recordings_list()
    
    def browse_source_dir(self):
        """浏览并选择源数据目录"""
        # 如果在处理中，不允许更改目录
        if self.is_processing:
            self.process_log.append("正在处理数据中，无法更改源目录")
            return
            
        dir_path = QFileDialog.getExistingDirectory(
            self, "选择源数据目录", self.source_dir_edit.text(),
            QFileDialog.ShowDirsOnly
        )
        
        if dir_path:
            self.source_dir_edit.setText(dir_path)
    
    def browse_output_dir(self):
        """浏览并选择输出目录"""
        # 如果在处理中，不允许更改目录
        if self.is_processing:
            self.process_log.append("正在处理数据中，无法更改输出目录")
            return
            
        dir_path = QFileDialog.getExistingDirectory(
            self, "选择输出目录", self.output_dir_edit.text(),
            QFileDialog.ShowDirsOnly
        )
        
        if dir_path:
            self.output_dir_edit.setText(dir_path)

    def browse_zip_dir(self):
        """浏览并选择ZIP保存目录"""
        # 如果在录制或压缩中，不允许更改目录
        if self.is_recording or self.is_compressing:
            self.log_message("正在录制或压缩中，无法更改ZIP保存目录", error=True)
            return
            
        dir_path = QFileDialog.getExistingDirectory(
            self, "选择ZIP保存目录", self.args.zip_dir,
            QFileDialog.ShowDirsOnly
        )
        
        if dir_path:
            self.zip_dir_edit.setText(dir_path)
            self.args.zip_dir = dir_path
            self.log_message(f"ZIP保存目录已更改为: {dir_path}")
    
    def save_settings(self):
        """保存设置"""
        # 如果在录制或压缩中，不允许保存设置
        if self.is_recording:
            self.log_message("正在录制中，无法保存设置", error=True)
            return
            
        # 更新参数
        self.args.save_dir = self.save_dir_edit.text()
        self.args.zip_dir = self.zip_dir_edit.text()  # 保存ZIP目录
        self.args.auto_zip = self.auto_zip_check.isChecked()
        self.args.delete_after_zip = self.delete_after_zip_check.isChecked()
        self.args.zip_compression = self.zip_level_combo.currentIndex()
        self.args.compress = self.compress_check.isChecked()
        self.args.ignore_missing_topics = self.ignore_missing_topics_check.isChecked()
        
        # 更新数据存储
        self.data_storage.base_dir = self.args.save_dir
        
        self.log_message("设置已保存")

    def start_data_processing(self):
        """开始数据处理"""
        # 检查是否在录制或压缩中
        if self.is_recording:
            self.process_log.append("正在录制数据中，无法同时进行数据处理")
            return
            
        if self.is_compressing:
            self.process_log.append("ZIP压缩进行中，无法同时进行数据处理")
            return
            
        if self.is_processing:
            self.process_log.append("已有数据处理任务在进行中")
            return
            
        # 获取源目录和输出目录
        source_dir = self.source_dir_edit.text()
        output_dir = self.output_dir_edit.text()
        
        # 检查目录是否存在
        if not os.path.exists(source_dir):
            self.process_log.append(f"源数据目录不存在: {source_dir}")
            return
            
        # 确保输出目录存在
        os.makedirs(output_dir, exist_ok=True)
        
        # 获取处理线程数
        max_workers = int(self.process_threads_combo.currentText())
        
        # 是否自动删除失败的处理结果
        delete_failed = self.delete_failed_check.isChecked()
        
        # 清空处理日志
        self.process_log.clear()
        self.process_log.append(f"开始处理数据...")
        self.process_log.append(f"源目录: {source_dir}")
        self.process_log.append(f"输出目录: {output_dir}")
        self.process_log.append(f"处理线程数: {max_workers}")
        self.process_log.append(f"自动删除失败结果: {'是' if delete_failed else '否'}")
        
        # 设置处理状态
        self.is_processing = True
        self.update_button_states()
        
        # 启动异步处理任务
        self.process_worker = DataProcessWorker(source_dir, output_dir, max_workers, delete_failed)
        self.process_worker.progress_signal.connect(self.on_process_progress)
        self.process_worker.finished_signal.connect(self.on_process_finished)
        self.process_worker.start()

    def on_process_progress(self, message):
        """处理进度更新"""
        self.process_log.append(message)
        # 滚动到底部
        self.process_log.verticalScrollBar().setValue(self.process_log.verticalScrollBar().maximum())

    def on_process_finished(self, success, results):
        """处理完成回调"""
        self.is_processing = False
        self.update_button_states()
        
        if success:
            self.process_log.append("\n====== 数据处理完成 ======")
            
            # 展示处理结果对话框
            dialog = ProcessResultDialog(results, self.output_dir_edit.text(), self)
            dialog.exec_()
        else:
            self.process_log.append("\n====== 数据处理失败 ======")

    def closeEvent(self, event):
        """关闭窗口事件处理"""
        if self.is_recording:
            self.stop_recording()
            
        # 如果在压缩中，询问是否等待
        if self.is_compressing:
            self.log_message("ZIP压缩正在进行中，请等待完成后再退出", error=True)
            event.ignore()
            return
            
        # 如果在处理数据中，询问是否等待
        if self.is_processing:
            reply = QMessageBox.question(self, '确认退出', 
                "数据处理正在进行中，确定要退出吗？", 
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            
            if reply == QMessageBox.No:
                event.ignore()
                return
            
        # 确保所有任务都已完成
        event.accept()

def get_arguments():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(description='ROS数据记录与处理系统参数配置')
    
    # 基础配置
    parser.add_argument('--save_dir', type=str, default="./recorded_data",
                      help='数据保存目录')
    parser.add_argument('--zip_dir', type=str, default="./recorded_data_zip",
                      help='ZIP文件保存目录，默认与数据保存目录相同')
    parser.add_argument('--frame_rate', type=int, default=30,
                      help='状态刷新帧率')
    parser.add_argument('--compress', action='store_true', default=True,
                      help='是否使用压缩图像话题')
    
    # 录制配置
    parser.add_argument('--auto_zip', action='store_true', default=True,
                      help='是否自动将录制数据打包为ZIP')
    parser.add_argument('--delete_after_zip', action='store_true', default=False,
                      help='创建ZIP后是否删除原始数据目录')
    parser.add_argument('--zip_compression', type=int, default=6,
                      help='ZIP压缩级别(0-9)，0表示不压缩，9表示最大压缩')
    parser.add_argument('--ignore_missing_topics', action='store_false', default=False,
                      help='忽略缺失的夹爪话题，允许在部分话题缺失时进行录制')
    
    # 话题配置
    parser.add_argument('--img_front_topic', type=str,
                      default='/camera_f/color/image_raw/compressed',
                      help='前方相机话题')
    parser.add_argument('--img_left_topic', type=str,
                      default='/camera_l/color/image_raw/compressed',
                      help='左侧相机话题')
    parser.add_argument('--img_right_topic', type=str,
                      default='/camera_r/color/image_raw/compressed',
                      help='右侧相机话题')
    parser.add_argument('--puppet_state_topic', type=str,
                      default='/puppet',
                      help='机器人状态话题')
    parser.add_argument('--gripper_left_topic', type=str,
                      default='/gripper1_position_mm_upsample',
                      help='左夹爪话题')
    parser.add_argument('--gripper_right_topic', type=str,
                      default='/gripper2_position_mm_upsample',
                      help='右夹爪话题')
    parser.add_argument('--gripper_action_topic', type=str,
                      default='/gripper_action',
                      help='夹爪控制指令话题')
    
    return parser.parse_args()

def main():
    """主函数"""
    # 解析参数
    args = get_arguments()
    
    # 确保所需的库已导入
    global h5py, QMetaObject, Q_ARG, cv2, AlohaCmd  # 确保全局可用
    import h5py
    from PyQt5.QtCore import QMetaObject, Q_ARG
    try:
        from PyQt5.QtCore import Qt  # Qt已经在其他地方导入
    except:
        pass
    import cv2
    try:
        from sdk.msg import AlohaCmd
    except ImportError:
        print("警告: 无法导入AlohaCmd消息类型，轨迹回放功能可能受限")
        # 创建一个简单的替代类
        class AlohaCmd:
            def __init__(self):
                self.arx_pos_left = []
                self.arx_pos_right = []
                self.cmd_left = 0
                self.cmd_right = 0
    
    # 创建QT应用
    app = QApplication(sys.argv)
    app.setStyle('Fusion')  # 使用Fusion风格，跨平台一致
    
    # 创建主窗口
    window = ROSRecorderGUI(args)
    window.show()
    
    # 运行应用
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
#python3 /root/collect_data_script/A0121_collect_data.py 