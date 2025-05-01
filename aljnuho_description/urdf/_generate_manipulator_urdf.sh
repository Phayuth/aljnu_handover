#!/bin/bash

ros2 run xacro xacro aljnu_ur5e_robotiq.urdf.xacro\
    safety_limits:=true \
    safety_pos_margin:=0.15 \
    safety_k_position:=20 \
    name:=ur \
    ur_type:=ur5e \
    tf_prefix:=ur5e_ > aljnu_ur5e_robotiq.urdf