#!/usr/bin/env python
# -*- coding: utf-8 -*-

from setuptools import setup, find_packages

setup(
    name="ros_data_collector",
    version="0.1.0",
    packages=find_packages(),
    install_requires=[
        "numpy",
        "pandas",
        "h5py",
        "tqdm",
        "opencv-python",
        "PyQt5",
    ],
    scripts=["scripts/collect_data.py"],
    author="ROS数据采集工具开发团队",
    author_email="example@example.com",
    description="用于ROS机器人数据采集、处理与回放的工具",
    keywords="ROS, data collection, robotics",
) 