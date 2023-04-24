
#include "tracking.hpp"

Tracking:: ~Tracking(){

void Tracking::track_center(const visualisation_msgs::MarkerArray, double trackin_array)

 tracking_array.push_back()
 
 if 

}



    <model name="moving_box">
      <pose frame=''>$5 5 5 0 0 0</pose>
      <link name='link'>
        <visual name='visual'>
          <geometry>
            <box>
              <size>2 2 2</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/DarkGray</name>
            </script>
          </material>
        </visual>
        <collision name='collision'>
          <geometry>
            <box>
              <size>2 2 2</size>
            </box>
          </geometry>
            <surface>
              <friction>
                <ode>
                  <mu>0</mu>
                  <mu2>0</mu2>
                </ode>
              </friction>
            </surface>
        </collision>
      </link>
      <plugin name="plannar_mover_plugin" filename="libplannar_mover_plugin.so"/>
    </model>