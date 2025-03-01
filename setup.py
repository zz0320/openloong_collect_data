from setuptools import setup, find_packages

with open('requirements.txt') as f:
    requirements = f.read().splitlines()

with open('README.md', 'r', encoding='utf-8') as f:
    long_description = f.read()

setup(
    name="openloong_collect_data",
    version="0.1.0",
    author="OpenLoong Team",
    author_email="openloong@example.com",
    description="OpenLoong机器人数据收集与处理系统",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/openloong/collect_data",
    packages=find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.6",
    install_requires=requirements,
    entry_points={
        'console_scripts': [
            'openloong-collector=openloong_collect_data.scripts.openloong_collect_data_gui:main',
        ],
    },
) 