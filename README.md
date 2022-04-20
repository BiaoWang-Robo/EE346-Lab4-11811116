# EE346 Lab4 lane following-11811116

This is the assignment of lab4 in EE346 by Biao Wang(11811116）

# Usage

## 1. Clone the source code

  cd ~/catkin_ws/src

  git clone https://github.com/jackwb00/EE346-Lab4-11811116.git

**PS: change the catkin_ws with your workspace's name**

## 2. Catkin make the lane following package

  cd ..

  catkin_make

## 3. Add course models

   export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/catkin_ws/src/lane_following/models

## 4. Launch the gazebo map

   source ~/catkin_ws/devel/setup.bash

   roslaunch lane_following race_track.launch 

## 5. Run lane following python node

#### Part1 normal lane following

   cd ~/catkin_ws/src/lane_following/scripts/

   chmod +x lane_following_part1.py

   cd ~/catkin_ws

   source devel/setup.bash

   rosrun lane_following lane_following_part1.py

#### Part2 lane following with BEV

   cd ~/catkin_ws/src/lane_following/scripts/

   chmod +x lane_following_part2.py

   cd ~/catkin_ws

   source devel/setup.bash

   rosrun lane_following lane_following_part2.py

#### Part3 lane following with BEV and aruco marker detection

   cd ~/catkin_ws/src/lane_following/scripts/

   chmod +x lane_following_part3.py

   cd ~/catkin_ws

   source devel/setup.bash

   rosrun lane_following lane_following_part3.py



## Reference

This project refers to the project https://github.com/zhaojieting/lane_following
