FROM ros:humble-ros-base
 
# Change the default shell to Bash
SHELL [ "/bin/bash" , "-c" ]
 
# Install Git and pip3
RUN apt-get update && apt-get install -y git python3-pip
 
# Create a ROS2 workspace and insall dependencies
RUN source /opt/ros/humble/setup.bash \
 && mkdir -p /ros2_ws/src \
 && cd /ros2_ws/src 

RUN apt-get install -y \
ros-humble-dynamixel-sdk \
ros-humble-dynamixel-sdk-custom-interfaces \
ros-humble-dynamixel-sdk-examples 
 
RUN pip3 install pyserial --user

# Copy the repo 
RUN cd /ros2_ws/src \
&& mkdir sine_pub
COPY ../ /ros2_ws/src/sine_pub

 # Build the ROS2 workspace and ensure it's sourced
RUN source /opt/ros/humble/setup.bash \
 && cd /ros2_ws \
 && colcon build --packages-select sine_pub
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc
RUN source /root/.bashrc

# Set the working folder at startup
#WORKDIR /ros2_ws#
#ENTRYPOINT [ "ros2" ]
#CMD ["ros2", "run", "sine_pub", "sine_pub"]

# Set the entry point
COPY ../dockerfile/ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
#CMD ["ros2", "run", "sine_pub", "sine_pub"]