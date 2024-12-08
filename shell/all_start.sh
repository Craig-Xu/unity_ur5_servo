#!/bin/bash 
# 0endpoint.sh  1gazebo.sh  2moveit.sh  3rviz.sh  4servo.sh
# 打开一个新的终端并执行指定的脚本，每个脚本都在一个新的终端标签页中运行,第一个sleep 1是为了等待终端打开


# gnome-terminal --window -e 'bash -c "bash scripts1.sh;exec bash"' \
# --tab -e 'bash -c "sleep 1; bash scripts2.sh;exec bash"' \
# --tab -e 'bash -c "sleep 2; bash scripts3.sh;exec bash"' \
# --tab -e 'bash -c "sleep 3; bash scripts4.sh;exec bash"' \

gnome-terminal --window -e 'bash -c "bash 0endpoint.sh;exec bash"' \
--tab -e 'bash -c "sleep 1; bash 1gazebo.sh;exec bash"' \
--tab -e 'bash -c "sleep 2; bash 2moveit.sh;exec bash"' \
--tab -e 'bash -c "sleep 3; bash 3rviz.sh;exec bash"' \
--tab -e 'bash -c "sleep 4; bash 4servo.sh;exec bash"' \
--tab -e 'bash -c "sleep 5; python3 ../teleop.py;exec bash"' \
