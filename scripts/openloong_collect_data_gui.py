#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
青龙ROS数据记录器和数据处理器主程序

用法: python openloong_collect_data_gui.py [参数]
"""

import os
import sys
import argparse
import signal
from PyQt5.QtWidgets import QApplication

# 解析命令行参数
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
    """程序入口点"""
    try:
        # 解析参数
        args = get_arguments()
        
        # 确保必要的目录存在
        os.makedirs(args.save_dir, exist_ok=True)
        if args.zip_dir and args.zip_dir != args.save_dir:
            os.makedirs(args.zip_dir, exist_ok=True)
        
        # 导入必要的依赖
        try:
            import rospy
            from openloong_collect_data.ui.main_window import ROSRecorderGUI
        except ImportError as e:
            print(f"错误: 导入依赖库失败 - {str(e)}")
            print("请确保已安装所有依赖项并且ROS环境已正确配置")
            sys.exit(1)
        
        # 设置信号处理
        def sigint_handler(*args):
            QApplication.quit()
        
        signal.signal(signal.SIGINT, sigint_handler)
        
        # 创建QT应用
        app = QApplication(sys.argv)
        app.setStyle('Fusion')  # 使用Fusion风格，跨平台一致
        
        # 创建主窗口
        window = ROSRecorderGUI(args)
        window.show()
        
        # 启动应用
        sys.exit(app.exec_())
    
    except KeyboardInterrupt:
        print("\n程序被用户中断")
        sys.exit(0)
    except Exception as e:
        print(f"程序启动失败: {str(e)}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main() 