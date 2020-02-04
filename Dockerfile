FROM ros:kinetic-ros-base
	
RUN apt-get update && apt-get install -y \
	g++ libffi-dev libopencv-dev libeigen3-dev libglu1-mesa-dev \
	libassimp-dev ros-kinetic-object-recognition-core exuberant-ctags \
	ros-kinetic-pcl-conversions python-rosinstall python-rosinstall-generator python-wstool build-essential\
	ros-kinetic-hardware-interface ros-kinetic-controller-manager \
	ros-kinetic-control-msgs ros-kinetic-transmission-interface \
	ros-kinetic-move-base-msgs ros-kinetic-smach* \
	ros-kinetic-geometric-shapes ros-kinetic-object-recognition-msgs \
	ros-kinetic-rviz ros-kinetic-moveit ros-kinetic-moveit-core ros-kinetic-control-toolbox \
	ros-kinetic-joint-limits-interface libsdl1.2-dev libosmesa6-dev libfreeimage3 libfreeimage-dev \
	libopenni-dev ros-kinetic-catkin ros-kinetic-ecto* ros-kinetic-opencv-candidate ros-kinetic-moveit-msgs \
	ros-kinetic-freenect-* ros-kinetic-rplidar-ros ros-kinetic-navigation \
	ros-kinetic-hector-slam ros-kinetic-amcl ros-kinetic-ros-control ros-kinetic-ros-controllers \
	ros-kinetic-rosserial ros-kinetic-rosserial-arduino ros-kinetic-dynamixel-controllers python-pip nano python-pyside less \
	&& pip install -U cffi && apt-get install -y ros-kinetic-robot-state-publisher && pip install -U couchapp

COPY ./couchdb/local.ini /etc/couchdb/local.ini	

COPY ./ws_hcr /ws_hcr

RUN . /opt/ros/$ROS_DISTRO/setup.sh; \
	cd /ws_hcr/src; \
	catkin_init_workspace; \
	cd /ws_hcr; \
	catkin_make; \
	echo "source /opt/ros/kinetic/setup.bash" >> /root/.bashrc; \
	echo "source /ws_hcr/devel/setup.bash" >> /root/.bashrc; \
	echo "export ROS_PACKAGE_PATH=/ws_hcr/src:/opt/ros/kinetic/share" >> /root/.bashrc