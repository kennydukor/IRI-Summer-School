FROM osrf/ros:melodic-desktop-full
SHELL ["/bin/bash", "-c"]
# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections
###Fix ROS not signed key
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
##Add IRI repository
RUN apt-get -qq update && apt-get -qq install -y apt-utils
RUN apt-get -qq install -y vim wget ca-certificates xsdcxx ssh-client gedit eog
RUN sudo sh -c 'echo "deb [arch=amd64] https://labrepo.iri.upc.edu/packages $(lsb_release -cs) main" > /etc/apt/sources.list.d/labrobotica_repo.list'
RUN wget -q -O - https://labrepo.iri.upc.edu/labrobotica_repo.gpg.key | sudo apt-key add -

RUN apt-get -qq update && apt-get -qq install -y iri-iriutils-dev iri-autonomous-driving-tools-dev iri-opendrive-road-map-dev

RUN cd /root
WORKDIR /root

RUN mkdir -p summer_school/catkin_ws/src
RUN mkdir -p summer_school/catkin_ws/rosinstall

RUN cd /root/summer_school/catkin_ws/rosinstall && wget -q https://gitlab.iri.upc.edu/labrobotica/ros/iri_ros_how_to/-/raw/master/rosinstall/iri_core.rosinstall
RUN cd /root/summer_school/catkin_ws/rosinstall && wget -q https://gitlab.iri.upc.edu/mobile_robotics/adc/platforms/model_car/iri_model_car_how_to/-/raw/master/rosinstall/iri_model_car_common.rosinstall
RUN cd /root/summer_school/catkin_ws/rosinstall && wget -q https://gitlab.iri.upc.edu/mobile_robotics/adc/platforms/model_car/iri_model_car_how_to/-/raw/master/rosinstall/iri_model_car_simulator.rosinstall
RUN cd /root/summer_school/catkin_ws/rosinstall && wget -q https://gitlab.iri.upc.edu/mobile_robotics/adc/adc_2021/iri_adc_simulation_workshop/-/raw/master/rosinstall/iri_adc.rosinstall
RUN cd /root/summer_school/catkin_ws/rosinstall && wget -q https://gitlab.iri.upc.edu/mobile_robotics/summer_school/cnn_workshop/iri_traffic_sign_cnn_workshop_how_to/-/raw/master/rosinstall/iri_traffic_sign_cnn_workshop.rosinstall
RUN cd /root/summer_school/catkin_ws/rosinstall && wget -q https://gitlab.iri.upc.edu/mobile_robotics/summer_school/navigation_workshop/iri_summer_school_nav_workshop_how_to/-/raw/master/rosinstall/iri_summer_school_nav_workshop.rosinstall

RUN cd /root/summer_school/catkin_ws/src && wstool init
RUN cd /root/summer_school/catkin_ws/src && wstool merge -y -r /root/summer_school/catkin_ws/rosinstall/iri_core.rosinstall
RUN cd /root/summer_school/catkin_ws/src && wstool merge -y -r /root/summer_school/catkin_ws/rosinstall/iri_model_car_common.rosinstall
RUN cd /root/summer_school/catkin_ws/src && wstool merge -y -r /root/summer_school/catkin_ws/rosinstall/iri_model_car_simulator.rosinstall
RUN cd /root/summer_school/catkin_ws/src && wstool merge -y -r /root/summer_school/catkin_ws/rosinstall/iri_adc.rosinstall
RUN cd /root/summer_school/catkin_ws/src && wstool set adc/ADC_2021/iri_sign_description --version-new summer_school -y
RUN cd /root/summer_school/catkin_ws/src && wstool merge -y -r /root/summer_school/catkin_ws/rosinstall/iri_traffic_sign_cnn_workshop.rosinstall
RUN cd /root/summer_school/catkin_ws/src && wstool merge -y -r /root/summer_school/catkin_ws/rosinstall/iri_summer_school_nav_workshop.rosinstall
RUN cd /root/summer_school/catkin_ws/src && wstool update

#manually download realsense2-description package, otherwise apt timeout and fails
RUN cd /var/cache/apt/archives && wget http://packages.ros.org/ros/ubuntu/pool/main/r/ros-melodic-realsense2-description/ros-melodic-realsense2-description_2.3.0-1bionic.20210601.162315_amd64.deb
RUN apt-get -qq install -y ros-melodic-realsense2-description 

RUN cd /root/summer_school/catkin_ws/src && rosdep install -q -y -i -r --from-paths .

RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && cd /root/summer_school/catkin_ws && catkin_make -DCATKIN_BLACKLIST_PACKAGES=''"
RUN echo "source /root/summer_school/catkin_ws/devel/setup.bash" >> /root/.bashrc
RUN /bin/bash -c "cd /root && source .bashrc"

RUN cd /root/summer_school && wget -q -O test2020.png "https://drive.google.com/uc?export=download&id=14pwD1jBnt6BG-QkPzPemC-FtrFLofC4R"

# RUN cd /root/summer_school && curl -H "Authorization: Bearer ya29.a0AfH6SMAR2AQprimulINPOHtK-1yL5ArAYTLVB3IAo4wxmtzBOhgt6ikd9tozjf0rwpzZCJgxzauU8RBthBR6X3JbOVa-Z3uVkjpwV8LgNmppYZDQvZ2OpYzQKKoGOY7j_OxGpdR8niY5S2MQpJjGfFYNaAD0" https://www.googleapis.com/drive/v3/files/1L1Bheu-KyGnT4wdV4ghdN42QIrZBlNX9?alt=media -o traffic_sign_data_set.tar.gz
# 
# RUN cd /root/summer_school && tar -zxvf traffic_sign_data_set.tar.gz

RUN apt-get -qq install -y python-pip virtualenv
RUN pip install -U pip
RUN pip install virtualenv

RUN cd /root/summer_school && virtualenv traffic_sign_venv && source traffic_sign_venv/bin/activate && pip install opencv-contrib-python numpy scikit-learn scikit-image imutils matplotlib tensorflow==2.0.0 pyyaml rospkg && deactivate

#Fix gazebo error/warning: [Err] [REST.cc:205] Error in REST request
#RUN cd && sed -i 's/ignitionfuel/ignitionrobotics/g' .ignition/fuel/config.yaml
