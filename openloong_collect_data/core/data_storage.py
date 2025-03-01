#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""数据存储模块

负责管理录制数据的存储、压缩和查询
"""

import os
import time
import csv
import zipfile
from datetime import datetime
from concurrent.futures import ThreadPoolExecutor
import numpy as np
import rospy
import cv2
from cv_bridge import CvBridge


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