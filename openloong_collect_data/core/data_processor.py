#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""数据处理器模块

实现对录制数据的同步、转换与重组功能
"""

import os
import glob
import shutil
from datetime import datetime
import pandas as pd
import numpy as np
import h5py
from tqdm import tqdm
from typing import Dict, List, Tuple, Any


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
        import csv
        
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