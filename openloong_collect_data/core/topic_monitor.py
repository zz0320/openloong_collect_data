#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""话题状态监控模块

负责监控和追踪ROS话题的状态，提供实时状态信息
"""

import time


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