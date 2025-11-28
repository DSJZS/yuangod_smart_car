**阅读前提概要**:
- 在位移和转角上默认使用米和弧度, 坐标系默认使用右手坐标系，遵循右手螺旋(比如沿着某轴逆时针旋转是该轴转角的正方向)
- 1 rad = 180 / π ≈ 57.2958 deg 
- 同名参数最好数值一致(尽管它们可能位于不同的插件之下)，除非特殊说明
- 由于Nav2支持插件，所以很多下述的很多参数的描述与存在只针对某个具体的插件
- 参数文件以 `params/nav2_params.yaml` 为主

**rviz**:  
- 使用nav2自带的rviz配置文件，在这个配置文件的基础上进行修改，确保nav2的功能可视化完全  
- 2D Pose Estimate   用于标记机器人在地图中(大概)的位姿
- Nav2 Goal          用于设置机器人的目标位姿以进行导航 

**插件调参备注**:  
- `amcl`，用于定位，启动时需要告知大致的位姿( rviz 中的 2D Pose Estimate 可完成这一点 )  
- `control_server` , 控制器
        `FollowPath`，路径跟踪，用于设置控制器插件(比如最大速度)
            DWB控制器:
                `xy_goal_tolerance`， xy 方向上位移的误差(2D不涉及z方向)
                `max*`，某种数值的最大值
                `min*`，某种数值的最小值
            MPPI控制器:
                
        `general_goal_checker`，用于设置到目标点精度(即机器人最终停止时的位姿与目标位姿可以相差多少)，
            `xy_goal_tolerance`， xy 方向上位移的误差(2D不涉及z方向) 
            `yaw_goal_tolerance`，yaw 方向上姿态的误差( 也可以理解为z轴转角的误差 )
            
- `velocity_smoother`，速度平滑器，也涉及各种速度的限制
- `*cost_map`, 全局或者局部代价地图，
        `robot_radius`，机器人的半径
        `inflation_radius`，代价地图的膨胀半径
