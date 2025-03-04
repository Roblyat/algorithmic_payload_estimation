# algorithmic_payload_estimation
algorithmic payload estimation ur5

# info
- robot description  -->  manipulator.urdf.xacro
- robot controller   -->  algorithmic_payload_estimation/hmi/include/hmi/RobotController.h
- payload estimation --> payload_estimation/scripts
                         - data preperation
                         - training
                         - prediction
                         - plots

# start
1.  - git clone git@github.com:Roblyat/algorithmic_payload_estimation.git
    - cd algorithmic_payload_estimation
    - git submodule update --init --recursive

2.  - xhost +local:docker
    - cd algorithmic_payload_estimation/docker
    - docker compose up --build

3.  - docker exec -it force_estimation_container /bin/bash
    - catkin_make
    - roslaunch manipulator_description manipulator_gazebo.launch