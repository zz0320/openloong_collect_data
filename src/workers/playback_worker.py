#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
轨迹回放线程
"""

import os
import time
import rospy
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QPixmap
from sensor_msgs.msg import JointState
try:
    from sdk.msg import AlohaCmd
except ImportError:
    # 创建一个简单的替代类
    class AlohaCmd:
        def __init__(self):
            self.arx_pos_left = []
            self.arx_pos_right = []
            self.cmd_left = 0
            self.cmd_right = 0


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