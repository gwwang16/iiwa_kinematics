#! /bin/bash
# This script safely launches ros nodes with buffer time to allow param server population
x-terminal-emulator -e roslaunch iiwa_arm target_description.launch &
sleep 3 &&
x-terminal-emulator -e roslaunch iiwa_arm iiwa_cafe.launch &
sleep 3 &&
x-terminal-emulator -e roslaunch iiwa_arm spawn_target.launch &
sleep 5 &&
#x-terminal-emulator -e roslaunch kuka_arm inverse_kinematics.launch
# roslaunch iiwa_arm inverse_kinematics.launch
