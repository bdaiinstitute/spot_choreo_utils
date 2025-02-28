#!/usr/bin/env bash
# Copyright (c) 2025 Boston Dynamics AI Institute LLC. All rights reserved

# Update the developer docker user to match the UID and GID of the host user
# This simplifies permissions with the mounted repo code
usermod -u $UID "developer"
groupmod -g $GID "developer"
chown -R "developer" /usr/local/lib/python3.10/dist-packages/
chown -R "developer" /dev/snd

# Pip install editable dependencies
cd /workspaces/spot_choreo_utils/external/spot_wrapper/
pip install -e .

# Install choreo utils
cd /workspaces/spot_choreo_utils/
pip install -e .

# Install web animator
cd /workspaces/spot_choreo_utils/spot_choreo_utils/web_animator/
bash ./scripts/install_prereqs
pip install -e .

cd /workspaces/spot_choreo_utils/spot_choreo_utils/web_animator/spot_web_animator/systems/external/spot_description
rm -rf build install log
ls
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 run xacro xacro -o ./urdf/out/spot.urdf ./urdf/spot.urdf.xacro include_transmissions:=true
ros2 run xacro xacro -o ./urdf/out/standalone_arm.urdf ./urdf/standalone_arm.urdf.xacro include_transmissions:=true gripperless:=true
ros2 run xacro xacro -o ./urdf/out/standalone_gripper.urdf ./urdf/standalone_gripper.urdf.xacro include_transmissions:=true


# enter the container as the newly configured user
/bin/bash -c "su - developer"
