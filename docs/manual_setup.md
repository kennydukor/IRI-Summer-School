# Manual environment setup

This workshop is mainly build around [ROS](https://www.ros.org/), the [Gazebo simulator](http://gazebosim.org/) (we use the version 9 of Gazebo that comes with ROS Melodic) and the [Tensorflow library](https://www.tensorflow.org/) with keras. These instructions will guide you through the process of setting up the environment necessary to complete the workshop.

This workshop has been tested in Linux Ubuntu 18.04, ROS Melodic and Tensorflow 2.0, and it is the recommended setup. Other version may also be compatible but they are not recommended to avoid unexpected problems.

## Install and setup ROS

If you are new to ROS, please first follow the next links to get a minimum understanding of what is ROS and how it works:

* [Introduction](http://wiki.ros.org/ROS/Introduction)
* [Basic concepts](http://wiki.ros.org/ROS/Concepts)
* [Advanced concepts](http://wiki.ros.org/ROS/Higher-Level%20Concepts)

To install ROS Melodic, follow the instructions in [here](http://wiki.ros.org/melodic/Installation/Ubuntu).

Make sure to install the **desktop-full** version to include the Gazebo simulator and all the graphic tools.

Make sure to follow **step 1.5** to setup the basic ROS environment and **step 1.6** to install the necessary tool to build new ROS packages.

## Install IRI dependencies:

```
sudo sh -c 'echo "deb [arch=amd64] https://labrepo.iri.upc.edu/packages $(lsb_release -cs) main" > /etc/apt/sources.list.d/labrobotica_repo.list'

wget -O - https://labrepo.iri.upc.edu/labrobotica_repo.gpg.key | sudo apt-key add -

sudo apt update

sudo apt install iri-iriutils-dev iri-autonomous-driving-tools-dev iri-opendrive-road-map-dev
```

## Create a ROS workspace. 
If you are new to ROS, first take a look at [this](http://wiki.ros.org/catkin/workspaces) to understand the concept of workspaces in ROS

If you have other workspaces in your computer, please make sure to first source the original ROS setup to avoid undesired workspace overlaying:

```
source /opt/ros/melodic/setup.bash
```

To create the workspace, execute the following commands

```
mkdir -p ~/summer_school/catkin_ws/src
cd ~/summer_school/catkin_ws
catkin_make
source devel/setup.bash
```
When the last command completes, the workspace is ready to be used.

To avoid sourcing the devel/setup.bash file in each new terminal, it is better to add it to the *bash.rc* file with the following command:
```
echo "source ~/summer_school/catkin_ws/devel/setup.bash" >> ~/.bashrc
```
On existing terminals, execute the following command to activate the new settings:
```
source ~/.bashrc
```
New terminals will already have the new settings.

It is recommended to use the default paths shown in the documentation throughout the workshop. If other paths are used, keep in mind to change them when necessary.

To simplify the process of including and managing packages in the workspace, the *wstool* tool will be used. Follow the instructions in [here](http://wiki.ros.org/wstool#Installation) to install it, and the instructions in [here](http://wiki.ros.org/wstool#Initialize_the_Workspace_Without_a_rosinstall_file) to set it up.

## Install the necessary ROS packages

Download the following rosinstall files:

  - [iri_core.rosinstall](https://gitlab.iri.upc.edu/labrobotica/ros/iri_ros_how_to/-/raw/master/rosinstall/iri_core.rosinstall)
  - [iri_model_car_common.rosinstall](https://gitlab.iri.upc.edu/mobile_robotics/adc/platforms/model_car/iri_model_car_how_to/-/raw/master/rosinstall/iri_model_car_common.rosinstall)
  - [iri_model_car_simulator.rosinstall](https://gitlab.iri.upc.edu/mobile_robotics/adc/platforms/model_car/iri_model_car_how_to/-/raw/master/rosinstall/iri_model_car_simulator.rosinstall)
  - [iri_adc.rosinstall](https://gitlab.iri.upc.edu/mobile_robotics/adc/adc_2021/iri_adc_simulation_workshop/-/raw/master/rosinstall/iri_adc.rosinstall)
  - [iri_traffic_sign_cnn_workshop.rosinstall](https://gitlab.iri.upc.edu/mobile_robotics/summer_school/cnn_workshop/iri_traffic_sign_cnn_workshop_how_to/-/raw/master/rosinstall/iri_traffic_sign_cnn_workshop.rosinstall)

```bash
roscd
cd ..
mkdir -p rosinstall
cd rosinstall
wget -q https://gitlab.iri.upc.edu/labrobotica/ros/iri_ros_how_to/-/raw/master/rosinstall/iri_core.rosinstall
wget -q https://gitlab.iri.upc.edu/mobile_robotics/adc/platforms/model_car/iri_model_car_how_to/-/raw/master/rosinstall/iri_model_car_common.rosinstall
wget -q https://gitlab.iri.upc.edu/mobile_robotics/adc/platforms/model_car/iri_model_car_how_to/-/raw/master/rosinstall/iri_model_car_simulator.rosinstall
wget -q https://gitlab.iri.upc.edu/mobile_robotics/adc/adc_2021/iri_adc_simulation_workshop/-/raw/master/rosinstall/iri_adc.rosinstall
wget -q https://gitlab.iri.upc.edu/mobile_robotics/summer_school/cnn_workshop/iri_traffic_sign_cnn_workshop_how_to/-/raw/master/rosinstall/iri_traffic_sign_cnn_workshop.rosinstall
```

and merge them into your workspace following the steps in this [tutorial](http://wiki.ros.org/wstool#Merge_in_Additional_rosinstall_Files).

```
roscd
cd ../src
wstool init
wstool merge ../rosinstall/iri_core.rosinstall
wstool merge ../rosinstall/iri_model_car_common.rosinstall
wstool merge ../rosinstall/iri_model_car_simulator.rosinstall
wstool merge ../rosinstall/iri_adc.rosinstall
wstool set adc/ADC_2021/iri_sign_description --version-new summer_school -y
wstool merge ../rosinstall/iri_traffic_sign_cnn_workshop.rosinstall
wstool update
```

- **iri_core** is a set of ROS C++ structures and tools developed at IRI intended to simplify the development of new ROS packages. Further information can be found [here](https://gitlab.iri.upc.edu/labrobotica/ros/iri_ros_how_to).

- **iri_model_car_common** is a set of ROS packages common to the real and simulated model car, a 1:8 scale car with several sensors which has been used for autonomous driving competitions. Further information can be found here [iri_model_car_how_to](https://gitlab.iri.upc.edu/mobile_robotics/adc/platforms/model_car/iri_model_car_how_to).

- **iri_model_car_simulator** is a set of ROS packages specific for the simulated model car.

- **iri_traffic_sign_cnn_workshop** is a set of ROS packages required to complete the workshop.

Install all ROS dependencies with the following commands:

```
roscd
cd ..
rosdep install -i -r -y --from-paths src
```

Compile all the ROS packages with the following commands:

```
roscd
cd ..
catkin_make
```

## Additional resources

### Road image file
In order to properly display the road in Gazebo and RVIZ, download the road image from [here](https://drive.google.com/file/d/14pwD1jBnt6BG-QkPzPemC-FtrFLofC4R/view?usp=sharing). It is recommended to keep the original name and to place it in the *summer_school* folder, but it is not mandatory. If an other name or path is chosen, remember to modify them in the ROS launch files. 

```
cd ~/summer_school
wget -O test2020.png "https://drive.google.com/uc?export=download&id=14pwD1jBnt6BG-QkPzPemC-FtrFLofC4R"
```

### CNN data set
Download the reduced data set used to train the CNN from this IRI Google Drive [link](https://drive.google.com/file/d/1CpWXKGTJBopcYqFFgesFDFDUAkzgLmbK/view?usp=sharing).

It is recommended to place it in the *~/summer_school* folder, but it can be placed anywhere. In that case, remember to change the path to the data set when launching the CNN training and testing scripts. From the download folder, execute the following command:

```
mv ~/Downloads/traffic_sign_data_set.tar.gz ~/summer_school
```

Extract the contents:
```
cd ~/summer_school
tar -zxvf traffic_sign_data_set.tar.gz
```

## Create a virtual environment for Tensorflow

The CNN used in this workshop is implemented using the Python version of the tensorflow library. To avoid any possible conflict with some Python libraries already installed in the system, it is recommended to create a python virtual environment. If you use anaconda or miniconda, please comment the associated lines in the *~/.bashrc* file to avoid any conflicts.

To install and setup the python virtual environment, follow the next steps:

* Install the pip application, if not already installed, and update it:
```
sudo apt install python-pip
pip install -U pip
```
* Install the virtual environment library. We use the simple virtualenv tool instead of anaconda because of ROS compatibility issues.
```
pip install virtualenv
```

* You may also need to install it with apt:
```
sudo apt install virtualenv
```

* Move to the root folder:
```
cd ~/summer_school
```
* Create the virtual environment:
```
virtualenv traffic_sign_venv
```
* Activate the virtual environment:
```
source traffic_sign_venv/bin/activate
```
* Install all the dependencies:
```
pip install opencv-contrib-python numpy scikit-learn scikit-image imutils matplotlib tensorflow==2.0.0 pyyaml rospkg
```
* Deactivate the virtual environment for now
```
deactivate
```
For this workshop, we use the CPU version of the Tensorflow library to make sure everybody can follow it. The image data set is small enough and the CNN used is simple enough to be trained in a few minutes. People familiar with CNN and the Tensorflow library may choose to use the GPU version to speed up the computations, however, it will not be supported in this workshop.
