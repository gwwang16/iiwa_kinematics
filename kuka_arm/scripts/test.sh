#! /bin/bash
# This script safely launches ros nodes with buffer time to allow param server population
x-terminal-emulator -e roslaunch kuka_arm robot_description.launch &
sleep 3 &&
x-terminal-emulator -e roslaunch kuka_arm robot_control.launch &
sleep 3 &&
# x-terminal-emulator -e roslaunch kr210_claw_moveit planning_context.launch &
sleep 5 &&
#x-terminal-emulator -e roslaunch kuka_arm inverse_kinematics.launch
# roslaunch kuka_arm inverse_kinematics.launch




