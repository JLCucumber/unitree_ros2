> This is a customed version README.
> For official REAMDME, please refer to [this](/dev/ros2_ws/src/unitree_ros2/README-official.md) file


1. Removed `cyclonedds_ws` directory
   No more nested ws.

```Bash
# Foxy Version
unitree_ros2/
├── cyclonedds_ws/          ← Foxy 用户需要
│   ├── src/
│   │   ├── cyclonedds/     ← 手动编译的 DDS 实现
│   │   ├── rmw_cyclonedds/ ← RMW 适配层
│   │   └── unitree/        ← 消息定义
│   └── install/
└── example/


# Humble Version
ros2_ws/src/
|── unitree_api/    ← 只需要消息定义
├── unitree_go/     ← 只需要消息定义
├── unitree_hg/     ← 只需要消息定义
└── unitree_examples/        ← 示例代码
```

2. Build Phase



3. Runtime Phase
