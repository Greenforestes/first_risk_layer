# 阶段2（共四个阶段）

- 设置动态障碍物到gazebo，设置启动动态障碍物launch脚本
- 发布障碍物两个当前状态话题和两个未来状态话题
- 设置risk_layer的local_costmap的插件，把四个话题中的/dynamic_agent/future_path接入到risk_layer
- baseline vs method对比





```
source install/setup.bash
# 加载世界模型以及机器人模型
ros2 launch agent_description gazebo_sim.launch.py 
```



```
ls /opt/ros/humble/lib | grep gazebo_ros_state

# 世界模型加载插件激活/gazebo/gazebo_ros_state话题
gedit ~/COFMRISKNVA/item_package/PRN/src/agent_description/world/custom_room.world

    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros>
        <namespace>/gazebo</namespace>
      </ros>
      <update_rate>20.0</update_rate>
    </plugin>
  </world>
</sdf>

# 加载设置变量
CYLINDER_SDF=/home/dewey/COFMRISKNVA/item_package/PRN/install/agent_dynamic_scene/share/agent_dynamic_scene/models/dynamic_cylinder/model.sdf
# 加载actor到gazebo
ros2 run gazebo_ros spawn_entity.py   -entity stage2_cylinder_1   -file $CYLINDER_SDF   -x 1.0   -y -1.0   -z 0.0
```



```
# blind_corner场景
    # 设置Base_line
        # 运行动态障碍物不同场景
        ros2 launch agent_dynamic_scene dynamic_scene.launch.py scenario_name:=blind_corner
        # 打开nav2导航以及rviz2界面
        ros2 launch agent_navigation2 navigation2_clean.launch.py
        # 开始点到点
        ros2 run agent_patrol fixed_trial_node --ros-args --params-file \
~/COFMRISKNVA/item_package/PRN/install/agent_patrol/share/agent_patrol/config/trial_blind_corner.yaml


    # 设置Risk_line
        # 运行动态障碍物不同场景
        ros2 launch agent_dynamic_scene dynamic_scene.launch.py scenario_name:=blind_corner
        # 打开nav2导航以及rviz2界面
        ros2 launch agent_navigation2 navigation2_risk_clean.launch.py
        # 开始点到点
        ros2 run agent_patrol fixed_trial_node --ros-args --params-file \
~/COFMRISKNVA/item_package/PRN/install/agent_patrol/share/agent_patrol/config/trial_blind_corner.yaml

---------------------------------------------------------------

# corridor_cross场景
	    # 设置Base_line
            # 运行动态障碍物不同场景
            ros2 launch agent_dynamic_scene dynamic_scene.launch.py scenario_name:=corridor_cross
            # future表示，进入risklayer，打开nav2导航以及rviz2界面
            ros2 launch agent_navigation2 navigation2_clean.launch.py
            # 开始点到点
            ros2 run agent_patrol fixed_trial_node --ros-args --params-file \
~/COFMRISKNVA/item_package/PRN/install/agent_patrol/share/agent_patrol/config/trial_corridor_cross.yaml
            



        # 设置Risk_line
            # 运行动态障碍物不同场景
            ros2 launch agent_dynamic_scene dynamic_scene.launch.py scenario_name:=corridor_cross
            # future表示，进入risklayer，打开nav2导航以及rviz2界面
            ros2 launch agent_navigation2 navigation2_risk_clean.launch.py
            # 开始点到点
            ros2 run agent_patrol fixed_trial_node --ros-args --params-file \
~/COFMRISKNVA/item_package/PRN/install/agent_patrol/share/agent_patrol/config/trial_corridor_cross.yaml


```



```
# 把 future 表示从“只影响 local 的软风险”升级成“同时影响 global 选路 + local 规避”的导航表示
# 精修版本PRN0-2-20260414,把AI输入接入NAV2，https://docs.nav2.org/configuration/packages/configuring-costmaps.html?utm_source=chatgpt.com
```



















