# Motion Retargeting Tool README
## 功能简介
拓展GMR用于将单个BVH动作文件重定向（retarget）到指定x-humanoid的机器人，并支持导出为不同数据源格式的动作文件。

## 快速使用
### 核心命令
```bash
python scripts/bvh_to_robot.py --bvh_file `BVH_ADDRESS` --robot `ROBOT` --save_path `SAVE_ADDRESS` --motion_fps `FPS` --format `NAME_OF_DATA_SOURCE` --add_collision_avoidance True
```

### 参数说明
| 参数名         | 说明                                                                 |
|----------------|----------------------------------------------------------------------|
| `BVH_ADDRESS`  | 待处理的BVH动作文件路径（必填）                                       |
| `ROBOT`        | 目标机器人模型名称（可选值：dex_evt2、tienkung2、tienkung2_lite、tienkung2_plus） |
| `SAVE_ADDRESS` | 重定向后动作文件的保存路径（必填）                                   |
| `FPS`          | 动作帧率（整数，如30、60、100）                                           |
| `NAME_OF_DATA_SOURCE` | 数据来源（可选值："xsens"、"noitom"、"lafan"）              |
| `add_collision_avoidance` | 是否检测身体部位的碰撞              |

### 示例命令
```bash
python scripts/bvh_to_robot.py --bvh_file data/noitom/8take219_chr00.bvh --robot dex_evt2 --save_path output/0320/8take219_chr00.pkl --motion_fps 100 --add_collision_avoidance True --format "noitom"

python scripts/bvh_to_robot.py --bvh_file data/lafan/run1_subject5.bvh --robot dex_evt2 --save_path output/0320/run1_subject5.pkl --motion_fps 60 --add_collision_avoidance True --format "lafan"
```

### 注意事项
1. 工具核心作用是将BVH动作重定向到指定机器人模型，需指定BVH路径、机器人类型、保存路径、帧率和数据格式；
2. `ROBOT` 参数仅支持tienkung系列机器人（dex_evt2, tienkung2PRO, tienkung2Lite, tienkung2Plus），`format` 参数目前支持xsens/noitom/lafan三种来源的数据，可根据数据的人体结构进行解析和配置json文件(详见下一章)；
3. 执行命令前需确保BVH文件路径正确，保存路径所在目录已创建。

### 解析动捕数据和运动学配置
#### 动捕数据 
位置在`general_motion_retargeting/utils/lafan1.py`
1. 使用任意可以打开动捕数据文件的软件(如：Blender https://www.blender.org/download/)，或直接用文本编辑器打开动捕数据文件，确认各个部位的名称。
2. 足部的信息可能是踝关节的position和脚趾的orientation组合，请自行判断
#### 运动学配置
json文件中，保存了需要使用到的配置信息，位置在`general_motion_retargeting/ik_configs`
| 参数名         | 说明                                                                 |
|----------------|----------------------------------------------------------------------|
| `robot_root_name`  | 机器人根节点 |
| `human_root_name`  | 动捕数据根节点 |
| `human_scale_table`  | 指定部位tf坐标到根节点的缩放                                       |
| `ground_height`        | 动捕数据的地面高度 |
| `ik_match_table`        | 跟踪任务的配置表格 |
| `use_ik_match_table1`        | 是否进行跟踪任务 |


```json
"ik_match_table1": {
        "pelvis": [ # 机器人 body 名称，请查阅对应机器人的xml或urdf文件
            "pelvis", # 对应动捕数据中的 body 名称, 此处以 "pelvis" 为例
            100, # weight to track 3D positions (xyz)
            10, # weight to track 3D rotations
            [
                0.0, # x offset added to human body "pelvis" x
                0.0, # y offset added to human body "pelvis" y
                0.0 # z offset added to human body "pelvis" z
            ],
            [
                # the rotation (represented as quaternion) applied to human body "pelvis". the order follows scalar first (wxyz)
                0.5,
                -0.5,
                -0.5,
                -0.5
            ]
        ],
      ...
```
