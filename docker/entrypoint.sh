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

# enter the container as the newly configured user
/bin/bash -c "su - developer"
