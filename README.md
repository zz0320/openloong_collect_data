# OpenLoong数据收集与处理系统

青龙ROS数据记录器和数据处理器，用于收集、处理和回放机器人轨迹数据。

## 主要功能

1. 通过GUI界面记录ROS话题数据
2. 对记录的数据进行同步、转换与重组
3. 生成标准格式的数据集
4. 回放记录的轨迹数据

## 安装

```bash
# 克隆仓库
git clone https://github.com/zz0320/openloong_collect_data.git
cd openloong_collect_data

# 安装依赖
pip install -r requirements.txt

# 安装包
pip install -e .
```

## 使用方法

### 启动图形界面

```bash
openloong-collector
```

或者

```bash
python scripts/openloong_collect_data_gui.py
```

## 系统要求

- Python 3.6+
- ROS 1 (已在ROS Melodic上测试)
- PyQt5

## 目录结构

```
openloong_collect_data/
│
├── openloong_collect_data/            # 主包目录
│   ├── core/                          # 核心功能模块
│   ├── ui/                            # UI界面模块
│   └── workers/                       # 工作线程模块
│
├── scripts/                           # 可执行脚本
├── setup.py                           # 安装脚本
└── requirements.txt                   # 依赖项
```

## 许可证

MIT 