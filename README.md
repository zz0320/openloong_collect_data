# ROS数据采集与处理系统

## 简介
这是一个用于ROS机器人数据采集、处理与轨迹回放的综合工具。项目使用模块化结构设计，将功能划分为多个模块，提高了代码可维护性和可扩展性。

## 主要功能
1. 通过GUI界面记录ROS话题数据
2. 对记录的数据进行同步、转换与重组
3. 生成标准格式的数据集
4. 回放机器人轨迹

## 项目结构
```
ros_data_collector/
├── setup.py               # 安装配置
├── requirements.txt       # 依赖项
├── README.md
├── scripts/
│   └── collect_data.py    # 主入口脚本
└── src/
    ├── core/              # 核心功能模块
    │   ├── data_processor.py  # 数据处理
    │   ├── data_storage.py    # 数据存储
    │   └── topic_monitor.py   # 话题监控
    ├── gui/               # GUI模块
    │   ├── main_window.py     # 主窗口
    │   └── dialogs/           # 对话框
    │       ├── trajectory_player.py  # 轨迹回放
    │       └── process_result.py     # 处理结果展示
    └── workers/           # 工作线程
        ├── playback_worker.py  # 回放线程
        ├── process_worker.py   # 处理线程
        └── zip_worker.py       # 压缩线程
```

## 依赖项
本项目依赖以下Python库：
- numpy
- pandas
- h5py
- tqdm
- opencv-python
- PyQt5
- ROS相关包

## 安装方法

### 1. 克隆仓库
```bash
git clone https://github.com/your-username/ros_data_collector.git
cd ros_data_collector
```

### 2. 安装依赖
```bash
pip install -r requirements.txt
```

### 3. 安装程序
```bash
python setup.py install
```

## 使用方法

### 基本用法
```bash
python scripts/collect_data.py
```

### 高级用法
```bash
# 指定数据保存目录和压缩选项
python scripts/collect_data.py --save_dir /path/to/save --compress

# 自动创建ZIP归档
python scripts/collect_data.py --auto_zip --delete_after_zip

# 设置自定义话题
python scripts/collect_data.py --img_front_topic /camera/front/image_raw
```

## 配置参数
可通过命令行参数设置以下选项：
- `--save_dir`: 数据保存目录
- `--zip_dir`: ZIP文件保存目录
- `--frame_rate`: 状态刷新帧率
- `--compress`: 是否使用压缩图像话题
- `--auto_zip`: 是否自动将录制数据打包为ZIP
- `--delete_after_zip`: 创建ZIP后是否删除原始数据目录
- `--zip_compression`: ZIP压缩级别(0-9)
- `--ignore_missing_topics`: 忽略缺失的话题，允许部分话题缺失时录制

## 工作流程
1. 启动程序，确保所有话题正常发布
2. 在"实时监控与录制"标签页开始录制
3. 完成录制后，可在"数据管理"标签页查看和管理录制结果
4. 使用"数据处理"标签页对录制数据进行处理
5. 在"轨迹回放"标签页回放处理后的数据

## 常见问题解答
1. **录制时没有收到图像数据**：检查相机话题是否正确发布，可能需要修改话题名称
2. **处理数据时报错**：确保录制的数据完整，包含所有必要的CSV文件和图像
3. **ZIP压缩失败**：检查磁盘空间是否足够，以及权限设置是否正确

## 模块开发说明
如果需要修改源码或添加新功能，请参考以下模块：
- `core/`: 包含核心功能实现，如数据处理、存储和话题监控
- `workers/`: 包含异步工作线程，用于处理耗时操作
- `gui/`: 包含所有GUI组件，主窗口和对话框实现

## 许可证
此项目遵循 MIT 许可证 