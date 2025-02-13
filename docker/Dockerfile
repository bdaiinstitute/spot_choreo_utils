# Copyright (c) 2025 Boston Dynamics AI Institute LLC. All rights reserved.
FROM ubuntu:22.04

USER root

# Setup python and notebooks
RUN apt-get update && apt-get install -y --no-install-recommends \
  python-is-python3 \
  python3-pip \ 
  vim \
  sudo \
  ffmpeg \
  git

ENV SETUP_DIR="/setup"
RUN mkdir /workspaces
RUN mkdir $SETUP_DIR

# Python deps
COPY requirements.txt $SETUP_DIR/requirements.txt
RUN pip install -r $SETUP_DIR/requirements.txt

# Create a non-root user 
ENV NEW_USER="developer"
RUN groupadd -g 1000 $NEW_USER
RUN useradd -d /workspaces/spot_choreo_utils  -s /bin/bash -m $NEW_USER -u 1000 -g 1000 && echo "$NEW_USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
RUN usermod -aG sudo $NEW_USER
RUN echo "$NEW_USER:ubuntu" | chpasswd

# Setup permissions
RUN chown -R $NEW_USER /workspaces
RUN chown -R $NEW_USER $SETUP_DIR
RUN chown -R $NEW_USER /usr/local/lib/python3.10/dist-packages/
COPY docker/entrypoint.sh $SETUP_DIR/entrypoint.sh

ENTRYPOINT ["/usr/bin/bash", "/setup/entrypoint.sh"]
CMD ["/bin/bash"]
