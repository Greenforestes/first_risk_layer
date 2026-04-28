# first_risk_layer
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
