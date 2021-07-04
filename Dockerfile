# Kudos to DOROWU for his amazing VNC 18.04 LXDE image
FROM dorowu/ubuntu-desktop-lxde-vnc:bionic

# Fix dirmngr
RUN sudo apt purge dirmngr -y && sudo apt update && sudo apt install dirmngr -y

# Adding keys for ROS
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Installing ROS
RUN sudo apt-get update && sudo apt-get install -y ros-melodic-desktop-full \
		wget git nano
RUN sudo apt-get install -y --no-install-recommends python-rosdep python-rosinstall python-rosinstall-generator python-wstool python-catkin-tools build-essential
RUN sudo apt-get install -y --no-install-recommends ros-melodic-xpp ros-melodic-ifopt libncurses5-dev
RUN rosdep init && rosdep update

RUN /bin/bash -c "echo 'export HOME=/home/ubuntu' >> /root/.bashrc && source /root/.bashrc"

# Creating ROS_WS
RUN mkdir -p ~/talos_public_ws/src

# Set up the workspace
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && \
                  cd ~/talos_public_ws/ && \
                  catkin_make && \
                  echo 'source ~/talos_public_ws/devel/setup.bash' >> ~/.bashrc && \
                  echo 'source ~/talos_public_ws/devel/setup.bash' >> /root/.bashrc "

# Updating ROSDEP and installing dependencies
#RUN cd ~/ros_ws && rosdep update && rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y

# Install the trajectory optimizer
RUN git clone https://github.com/noctrog/towr -b talos ~/talos_public_ws/src/towr

# Sourcing
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && \
                  cd ~/talos_public_ws/ && rm -rf build devel && \
                  catkin build --force-cmake"

# Install Talos workspace
RUN cd ~/talos_public_ws && wget https://raw.githubusercontent.com/pal-robotics/talos_tutorials/kinetic-devel/talos_public-melodic.rosinstall
RUN cd ~/talos_public_ws && rosinstall src /opt/ros/melodic talos_public-melodic.rosinstall
RUN rm /etc/ros/rosdep/sources.list.d/20-default.list
RUN cd ~/talos_public_ws && rosdep init && rosdep fix-permissions && rosdep update
RUN cd ~/talos_public_ws && rosdep install -y --from-paths src --ignore-src --rosdistro melodic --skip-keys="math_utils opencv2 pal_laser_filters speed_limit_node sensor_to_cloud hokuyo_node libdw-dev gmock walking_utils rqt_current_limit_controller simple_grasping_action reemc_init_offset_controller walking_controller current_limit_controller urdf_test eigen_checks rbdl pal_ros_utils"
RUN apt-get install -y --no-install-recommends bear
RUN . /opt/ros/melodic/setup.sh && cd ~/talos_public_ws && bear catkin build --force-cmake -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Install Pinocchio
RUN echo "deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | sudo tee /etc/apt/sources.list.d/robotpkg.list
RUN curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add -
RUN apt-get update
RUN apt-get install -qqy robotpkg-py27-pinocchio
RUN /bin/bash -c "echo 'export PATH=/opt/openrobots/bin:$PATH' >> ~/.bashrc"
RUN /bin/bash -c "echo 'export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH' >> ~/.bashrc"
RUN /bin/bash -c "echo 'export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH' >> ~/.bashrc"
RUN /bin/bash -c "echo 'export PYTHONPATH=/opt/openrobots/lib/python2.7/site-packages:$PYTHONPATH # Adapt your desired python version here' >> ~/.bashrc"
RUN /bin/bash -c "echo 'export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH' >> ~/.bashrc"
RUN /bin/bash -c "echo 'export PATH=/opt/openrobots/bin:$PATH' >> /root/.bashrc"
RUN /bin/bash -c "echo 'export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH' >> /root/.bashrc"
RUN /bin/bash -c "echo 'export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH' >> /root/.bashrc"
RUN /bin/bash -c "echo 'export PYTHONPATH=/opt/openrobots/lib/python2.7/site-packages:$PYTHONPATH # Adapt your desired python version here' >> /root/.bashrc"
RUN /bin/bash -c "echo 'export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH' >> /root/.bashrc"

# Install controller dependencies
RUN apt-get install -y --no-install-recommends \
    ros-melodic-ackermann-msgs \
    ros-melodic-ackermann-steering-controller \
    ros-melodic-ackermann-steering-controller-dbgsym \
    ros-melodic-actionlib \
    ros-melodic-actionlib-dbgsym \
    ros-melodic-actionlib-lisp \
    ros-melodic-actionlib-msgs \
    ros-melodic-actionlib-tutorials \
    ros-melodic-actionlib-tutorials-dbgsym \
    ros-melodic-adi-driver \
    ros-melodic-adi-driver-dbgsym \
    ros-melodic-agni-tf-tools \
    ros-melodic-agni-tf-tools-dbgsym \
    ros-melodic-allocators \
    ros-melodic-angles \
    ros-melodic-app-manager \
    ros-melodic-ariles-ros \
    ros-melodic-asr-msgs \
    ros-melodic-assimp-devel \
    ros-melodic-assimp-devel-dbgsym \
    ros-melodic-assisted-teleop \
    ros-melodic-assisted-teleop-dbgsym \
    ros-melodic-astuff-sensor-msgs \
    ros-melodic-async-comm \
    ros-melodic-async-comm-dbgsym \
    ros-melodic-async-web-server-cpp \
    ros-melodic-async-web-server-cpp-dbgsym \
    ros-melodic-ati-force-torque \
    ros-melodic-ati-force-torque-dbgsym \
    ros-melodic-backward-ros \
    ros-melodic-backward-ros-dbgsym \
    ros-melodic-bagger \
    ros-melodic-bagger-dbgsym \
    ros-melodic-baldor \
    ros-melodic-boost-sml \
    ros-melodic-boost-sml-dbgsym \
    ros-melodic-brics-actuator \
    ros-melodic-calibration \
    ros-melodic-calibration-estimation \
    ros-melodic-calibration-launch \
    ros-melodic-calibration-msgs \
    ros-melodic-calibration-setup-helper \
    ros-melodic-capabilities \
    ros-melodic-catkin \
    ros-melodic-catkin-pip \
    ros-melodic-catkin-virtualenv \
    ros-melodic-chomp-motion-planner \
    ros-melodic-chomp-motion-planner-dbgsym \
    ros-melodic-cl-tf \
    ros-melodic-cl-tf2 \
    ros-melodic-cl-transforms \
    ros-melodic-cl-transforms-stamped \
    ros-melodic-cl-urdf \
    ros-melodic-cl-utils \
    ros-melodic-class-loader \
    ros-melodic-class-loader-dbgsym \
    ros-melodic-cmake-modules \
    ros-melodic-code-coverage \
    ros-melodic-codec-image-transport \
    ros-melodic-codec-image-transport-dbgsym \
    ros-melodic-collada-parser \
    ros-melodic-collada-parser-dbgsym \
    ros-melodic-collada-urdf \
    ros-melodic-collada-urdf-dbgsym \
    ros-melodic-collada-urdf-jsk-patch \
    ros-melodic-collada-urdf-jsk-patch-dbgsym \
    ros-melodic-combined-robot-hw \
    ros-melodic-combined-robot-hw-dbgsym \
    ros-melodic-combined-robot-hw-tests \
    ros-melodic-combined-robot-hw-tests-dbgsym \
    ros-melodic-common-msgs \
    ros-melodic-common-tutorials \
    ros-melodic-computer-status-msgs \
    ros-melodic-concert-msgs \
    ros-melodic-concert-service-msgs \
    ros-melodic-concert-workflow-engine-msgs \
    ros-melodic-control-box-rst \
    ros-melodic-control-msgs \
    ros-melodic-control-toolbox \
    ros-melodic-control-toolbox-dbgsym \
    ros-melodic-controller-interface \
    ros-melodic-controller-manager \
    ros-melodic-controller-manager-dbgsym \
    ros-melodic-controller-manager-msgs \
    ros-melodic-controller-manager-tests \
    ros-melodic-controller-manager-tests-dbgsym \
    ros-melodic-convex-decomposition \
    ros-melodic-convex-decomposition-dbgsym \
    ros-melodic-cpp-common \
    ros-melodic-cpp-common-dbgsym \
    ros-melodic-cpr-multimaster-tools \
    ros-melodic-criutils \
    ros-melodic-ddynamic-reconfigure \
    ros-melodic-ddynamic-reconfigure-dbgsym \
    ros-melodic-ddynamic-reconfigure-python \
    ros-melodic-derived-object-msgs \
    ros-melodic-desktop-full \
    ros-melodic-diff-drive-controller \
    ros-melodic-diff-drive-controller-dbgsym \
    ros-melodic-driver-base \
    ros-melodic-driver-common \
    ros-melodic-dual-quaternions \
    ros-melodic-dual-quaternions-ros \
    ros-melodic-effort-controllers \
    ros-melodic-effort-controllers-dbgsym \
    ros-melodic-eigen-conversions \
    ros-melodic-eigen-conversions-dbgsym \
    ros-melodic-eigen-stl-containers \
    ros-melodic-eigenpy \
    ros-melodic-eigenpy-dbgsym \
    ros-melodic-eiquadprog \
    ros-melodic-eiquadprog-dbgsym \
    ros-melodic-fcl-catkin \
    ros-melodic-fcl-catkin-dbgsym \
    ros-melodic-fetchit-challenge \
    ros-melodic-force-torque-sensor \
    ros-melodic-force-torque-sensor-controller \
    ros-melodic-force-torque-sensor-controller-dbgsym \
    ros-melodic-force-torque-sensor-dbgsym \
    ros-melodic-forward-command-controller \
    ros-melodic-four-wheel-steering-controller \
    ros-melodic-four-wheel-steering-controller-dbgsym \
    ros-melodic-four-wheel-steering-msgs \
    ros-melodic-gateway-msgs \
    ros-melodic-gazebo-dev \
    ros-melodic-gazebo-msgs \
    ros-melodic-gazebo-plugins \
    ros-melodic-gazebo-plugins-dbgsym \
    ros-melodic-gazebo-ros \
    ros-melodic-gazebo-ros-control \
    ros-melodic-gazebo-ros-control-dbgsym \
    ros-melodic-gazebo-ros-dbgsym \
    ros-melodic-gazebo-ros-pkgs \
    ros-melodic-gazebo-video-monitor-plugins \
    ros-melodic-gazebo-video-monitor-plugins-dbgsym \
    ros-melodic-gdrive-ros \
    ros-melodic-gencpp \
    ros-melodic-generic-throttle \
    ros-melodic-geneus \
    ros-melodic-genlisp \
    ros-melodic-genmsg \
    ros-melodic-gennodejs \
    ros-melodic-genpy \
    ros-melodic-geodesy \
    ros-melodic-geodesy-dbgsym \
    ros-melodic-geographic-info \
    ros-melodic-geographic-msgs \
    ros-melodic-geometric-shapes \
    ros-melodic-geometric-shapes-dbgsym \
    ros-melodic-geometry \
    ros-melodic-geometry-msgs \
    ros-melodic-geometry-tutorials \
    ros-melodic-geometry2 \
    ros-melodic-geos-cmake-module \
    ros-melodic-gl-dependency \
    ros-melodic-h264-encoder-core \
    ros-melodic-h264-encoder-core-dbgsym \
    ros-melodic-h264-video-encoder \
    ros-melodic-h264-video-encoder-dbgsym \
    ros-melodic-hector-gazebo \
    ros-melodic-hector-gazebo-plugins \
    ros-melodic-hector-gazebo-plugins-dbgsym \
    ros-melodic-hector-gazebo-thermal-camera \
    ros-melodic-hector-gazebo-thermal-camera-dbgsym \
    ros-melodic-hector-gazebo-worlds \
    ros-melodic-hector-geotiff \
    ros-melodic-hector-geotiff-dbgsym \
    ros-melodic-hector-geotiff-plugins \
    ros-melodic-hector-geotiff-plugins-dbgsym \
    ros-melodic-hector-imu-attitude-to-tf \
    ros-melodic-hector-imu-attitude-to-tf-dbgsym \
    ros-melodic-hector-imu-tools \
    ros-melodic-hector-imu-tools-dbgsym \
    ros-melodic-hector-map-server \
    ros-melodic-hector-map-server-dbgsym \
    ros-melodic-hector-map-tools \
    ros-melodic-hector-mapping \
    ros-melodic-hector-mapping-dbgsym \
    ros-melodic-hector-marker-drawing \
    ros-melodic-hector-models \
    ros-melodic-hector-nav-msgs \
    ros-melodic-hector-sensors-description \
    ros-melodic-hector-sensors-gazebo \
    ros-melodic-hector-slam \
    ros-melodic-hector-slam-launch \
    ros-melodic-hector-trajectory-server \
    ros-melodic-hector-trajectory-server-dbgsym \
    ros-melodic-hector-xacro-tools \
    ros-melodic-heron-control \
    ros-melodic-heron-description \
    ros-melodic-heron-gazebo \
    ros-melodic-heron-msgs \
    ros-melodic-heron-simulator \
    ros-melodic-hfl-driver \
    ros-melodic-hfl-driver-dbgsym \
    ros-melodic-hls-lfcd-lds-driver \
    ros-melodic-hls-lfcd-lds-driver-dbgsym \
    ros-melodic-hpp-fcl \
    ros-melodic-hpp-fcl-dbgsym \
    ros-melodic-ieee80211-channels \
    ros-melodic-imu-complementary-filter \
    ros-melodic-imu-complementary-filter-dbgsym \
    ros-melodic-imu-filter-madgwick \
    ros-melodic-imu-filter-madgwick-dbgsym \
    ros-melodic-imu-monitor \
    ros-melodic-imu-pipeline \
    ros-melodic-imu-processors \
    ros-melodic-imu-processors-dbgsym \
    ros-melodic-imu-sensor-controller \
    ros-melodic-imu-sensor-controller-dbgsym \
    ros-melodic-imu-tools \
    ros-melodic-imu-transformer \
    ros-melodic-imu-transformer-dbgsym \
    ros-melodic-joint-limits-interface \
    ros-melodic-joint-qualification-controllers \
    ros-melodic-joint-qualification-controllers-dbgsym \
    ros-melodic-joint-state-controller \
    ros-melodic-joint-state-controller-dbgsym \
    ros-melodic-joint-state-publisher \
    ros-melodic-joint-state-publisher-gui \
    ros-melodic-joint-states-settler \
    ros-melodic-joint-states-settler-dbgsym \
    ros-melodic-joint-trajectory-action \
    ros-melodic-joint-trajectory-action-dbgsym \
    ros-melodic-joint-trajectory-action-tools \
    ros-melodic-joint-trajectory-controller \
    ros-melodic-joint-trajectory-controller-dbgsym \
    ros-melodic-joint-trajectory-generator \
    ros-melodic-joint-trajectory-generator-dbgsym \
    ros-melodic-jointstick \
    ros-melodic-joy \
    ros-melodic-joy-dbgsym \
    ros-melodic-joy-listener \
    ros-melodic-joy-teleop \
    ros-melodic-joystick-drivers \
    ros-melodic-joystick-interrupt \
    ros-melodic-joystick-interrupt-dbgsym \
    ros-melodic-json-msgs \
    ros-melodic-json-transport \
    ros-melodic-kdl-conversions \
    ros-melodic-kdl-conversions-dbgsym \
    ros-melodic-kdl-parser \
    ros-melodic-kdl-parser-dbgsym \
    ros-melodic-kdl-parser-py \
    ros-melodic-key-teleop \
    ros-melodic-libcreate \
    ros-melodic-message-filters \
    ros-melodic-message-filters-dbgsym \
    ros-melodic-message-generation \
    ros-melodic-message-relay \
    ros-melodic-message-relay-dbgsym \
    ros-melodic-message-runtime \
    ros-melodic-message-to-tf \
    ros-melodic-message-to-tf-dbgsym \
    ros-melodic-moveit \
    ros-melodic-moveit-chomp-optimizer-adapter \
    ros-melodic-moveit-chomp-optimizer-adapter-dbgsym \
    ros-melodic-moveit-commander \
    ros-melodic-moveit-controller-manager-example \
    ros-melodic-moveit-controller-manager-example-dbgsym \
    ros-melodic-moveit-core \
    ros-melodic-moveit-core-dbgsym \
    ros-melodic-moveit-experimental \
    ros-melodic-moveit-fake-controller-manager \
    ros-melodic-moveit-fake-controller-manager-dbgsym \
    ros-melodic-moveit-kinematics \
    ros-melodic-moveit-kinematics-dbgsym \
    ros-melodic-moveit-msgs \
    ros-melodic-moveit-opw-kinematics-plugin \
    ros-melodic-moveit-opw-kinematics-plugin-dbgsym \
    ros-melodic-moveit-planners \
    ros-melodic-moveit-planners-chomp \
    ros-melodic-moveit-planners-chomp-dbgsym \
    ros-melodic-moveit-planners-ompl \
    ros-melodic-moveit-planners-ompl-dbgsym \
    ros-melodic-moveit-plugins \
    ros-melodic-moveit-pr2 \
    ros-melodic-moveit-python \
    ros-melodic-moveit-resources \
    ros-melodic-moveit-resources-fanuc-description \
    ros-melodic-moveit-resources-fanuc-moveit-config \
    ros-melodic-moveit-resources-panda-description \
    ros-melodic-moveit-resources-panda-moveit-config \
    ros-melodic-moveit-resources-pr2-description \
    ros-melodic-moveit-ros \
    ros-melodic-moveit-ros-benchmarks \
    ros-melodic-moveit-ros-benchmarks-dbgsym \
    ros-melodic-moveit-ros-control-interface \
    ros-melodic-moveit-ros-control-interface-dbgsym \
    ros-melodic-moveit-ros-manipulation \
    ros-melodic-moveit-ros-manipulation-dbgsym \
    ros-melodic-moveit-ros-move-group \
    ros-melodic-moveit-ros-move-group-dbgsym \
    ros-melodic-moveit-ros-occupancy-map-monitor \
    ros-melodic-moveit-ros-occupancy-map-monitor-dbgsym \
    ros-melodic-moveit-ros-perception \
    ros-melodic-moveit-ros-perception-dbgsym \
    ros-melodic-moveit-ros-planning \
    ros-melodic-moveit-ros-planning-dbgsym \
    ros-melodic-moveit-ros-planning-interface \
    ros-melodic-moveit-ros-planning-interface-dbgsym \
    ros-melodic-moveit-ros-robot-interaction \
    ros-melodic-moveit-ros-robot-interaction-dbgsym \
    ros-melodic-moveit-ros-visualization \
    ros-melodic-moveit-ros-visualization-dbgsym \
    ros-melodic-moveit-ros-warehouse \
    ros-melodic-moveit-ros-warehouse-dbgsym \
    ros-melodic-moveit-runtime \
    ros-melodic-moveit-servo \
    ros-melodic-moveit-servo-dbgsym \
    ros-melodic-moveit-setup-assistant \
    ros-melodic-moveit-setup-assistant-dbgsym \
    ros-melodic-moveit-sim-controller \
    ros-melodic-moveit-sim-controller-dbgsym \
    ros-melodic-moveit-simple-controller-manager \
    ros-melodic-moveit-simple-controller-manager-dbgsym \
    ros-melodic-moveit-visual-tools \
    ros-melodic-moveit-visual-tools-dbgsym \
    ros-melodic-movie-publisher \
    ros-melodic-mpc-local-planner \
    ros-melodic-mpc-local-planner-dbgsym \
    ros-melodic-mpc-local-planner-examples \
    ros-melodic-mpc-local-planner-msgs \
    ros-melodic-nlopt \
    ros-melodic-nlopt-dbgsym \
    ros-melodic-orocos-kdl \
    ros-melodic-orocos-kdl-dbgsym \
    ros-melodic-orocos-kinematics-dynamics \
    ros-melodic-ps3joy \
    ros-melodic-ps3joy-dbgsym \
    ros-melodic-robot \
    ros-melodic-robot-activity \
    ros-melodic-robot-activity-dbgsym \
    ros-melodic-robot-activity-msgs \
    ros-melodic-robot-activity-tutorials \
    ros-melodic-robot-activity-tutorials-dbgsym \
    ros-melodic-robot-body-filter \
    ros-melodic-robot-body-filter-dbgsym \
    ros-melodic-robot-calibration \
    ros-melodic-robot-calibration-dbgsym \
    ros-melodic-robot-calibration-msgs \
    ros-melodic-robot-controllers \
    ros-melodic-robot-controllers-dbgsym \
    ros-melodic-robot-controllers-interface \
    ros-melodic-robot-controllers-interface-dbgsym \
    ros-melodic-robot-controllers-msgs \
    ros-melodic-robot-localization \
    ros-melodic-robot-localization-dbgsym \
    ros-melodic-robot-mechanism-controllers \
    ros-melodic-robot-mechanism-controllers-dbgsym \
    ros-melodic-robot-navigation \
    ros-melodic-robot-one \
    ros-melodic-robot-one-dbgsym \
    ros-melodic-robot-pose-ekf \
    ros-melodic-robot-pose-ekf-dbgsym \
    ros-melodic-robot-self-filter \
    ros-melodic-robot-self-filter-dbgsym \
    ros-melodic-robot-setup-tf-tutorial \
    ros-melodic-robot-setup-tf-tutorial-dbgsym \
    ros-melodic-robot-state-publisher \
    ros-melodic-robot-state-publisher-dbgsym \
    ros-melodic-robot-statemachine \
    ros-melodic-robot-upstart \
    ros-melodic-roboticsgroup-upatras-gazebo-plugins \
    ros-melodic-roboticsgroup-upatras-gazebo-plugins-dbgsym \
    ros-melodic-xsens-driver \
