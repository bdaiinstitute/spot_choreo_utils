# spot_choreo_utils
This repository enables programatically building, editing, and playing back choreography on Spot. It's built on top of the [Python Choreography Client](https://github.com/boston-dynamics/spot-sdk/tree/master/python/bosdyn-choreography-client/src/bosdyn/choreography/client) and allows programmers to directly interact with the [SDK and animations](https://dev.bostondynamics.com/docs/concepts/choreography/readme#) without requiring the Choreographer Graphical User Interface.

Users can:
- Programatically build animations and choreography sequences
- Build animations by posing the physical robot (using the spot tablet or other control interface) and save the robot's joint angles as a keyframe pose
- Build animations by posing a robot through a web interface and playing back on robot
- Play choreography sequences on multiple robots synchronized with music

See the tutorials folder for walk throughs on these topics.

### Repo Structure
- choreo_files: storage for example animations and choreography files, and default place that new animations are stored (gitignored)
- docker: contains dockerfiles and scripts to setup a basic environment for choreography development
- external: git submodule dependencies for the spot_chore_utils repo
- spot_choreo_utils: core library
    - test: Unit tests for the library
- tutorials: jupyter notebook tutorials of how to use spot_choreo_utils

# Installation
This repository uses git submodules and Docker to manage dependencies. The easiest way to start working with spot_choreo_utils is to:
  - git submodule init
  - git submodule update
  - cd docker
  - python start_docker.py

This will download all external dependency repositories, build a docker image and start a new container with all dependencies installed. The repository is passed into the docker image as a volume so that changes will automatically pass through and persist on disk.

For a full dependencies list see the Dockerfile and entrypoint scripts.

## Setting up the Web Animator
The Web Animator provides a way to create pose-to-pose animations through the web browser.
  - cd spot_choreo_utils/web_animator
  - pip install -e .


## Animating with the Web Animator
You can explore potential choreographic poses and export them for testing on the robot through the Web Animator. The Web Animator does its best to guarantee stability and on robot validity when all feet are locked on the ground, but once feet are unlocked there is a much heavier burden placed on the choreographer to think about stability and comply with choreographer protobuf requirements.

To start animating:
  - cd spot_choreo_utils/web_animator/spot_web_animator
  - python animate.py
  - Type an animation name into the terminal 
  - Open a seperate tab to localhost:7000 for web visualizer

Use the animation sliders to set robot poses, and then use the Save Pose As Keyframe button to add the pose to an animation. The animation you create will be saved to the active directory under choreo_files.

# Contributing to this repo
This repository enforces `ruff` and `black` linting. To verify that your code will pass inspection, install `pre-commit` and run:
```bash
pre-commit install
pre-commit run --all-files
```
The [Google Style Guide](https://google.github.io/styleguide/) is followed for default formatting. 

### Testing 
The spot_choreo_utils library uses pytest for unit testing. Prior to submitting code please add unit tests and verify that all tests pass with pytest
- start the docker
- pytest

### Generating Protos
If you change the proto definitions, re-generate the pb2.py definitions with: 
- cd spot_choreo_utils/spot_choreo_utils/protos
- python regenerate_protos.py
