# CNN Workshop

In this workshop we will demonstrate the use of a Convolutional Neural Network (CNN) to identify traffic signs and act accordingly in a simulated environment. It is not intended as an in depth study of CNN, it is designed to apply a very powerful tool in a more *practical* set up, beyond the classification rates and the image data sets.

- We will use Linux [Ubuntu](https://ubuntu.com/) 18.04 LTS and it’s associated [ROS](https://www.ros.org/) Melodic version.
  - Newer versions haven’t been tested. 
- If you are new to Linux/Ubuntu, you may find it helpful to first do a quick tutorial on common command line tools for Linux. For example, [here](http://www.ee.surrey.ac.uk/Teaching/Unix/).
- If you are new to ROS, we encourage you to follow the [ROS Introduction](http://wiki.ros.org/ROS/Introduction), including the reading of its [Concepts](http://wiki.ros.org/ROS/Concepts) and doing its [Tutorials](http://wiki.ros.org/ROS/Tutorials)
- We will use the Gazebo simulator (version 9.0 coming with ROS Melodic). This simulator has some picky CPU/GPU requirements, so it’s better to use a modern computer.
- For any questions or comments, use the workshop channel in the Slack workspace.

This workshop has the following structure:

1. [Setup the environment using Docker](docker/docker_image_installation.md).
    * Alternatively you can do a [manual installation](docs/manual_setup.md). 
2. [Get familiar with the simulated environment](docs/simulated_car.md).
3. [Get familiar with the CNN used to classify traffic signs](docs/traffic_sign_classifier_cnn.md).
4. [Put it all together](docs/ros_and_cnn.md).
5. [Exercises](docs/exercises.md).

For any questions or comments, use the #workshop-day1 Slack channel provided by the organization.
