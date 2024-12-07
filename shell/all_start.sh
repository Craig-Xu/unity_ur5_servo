# 0endpoint.sh  1gazebo.sh  2moveit.sh  3rviz.sh  4servo.sh gnome 终端新建tab启动


gnome-terminal --tab -- bash -c "sh 0endpoint.sh && sh 1gazebo.sh && sh 2moveit.sh && sh 3rviz.sh && sh 4servo.sh; exec bash"
