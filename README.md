# CMPUT 412 Winter 2019 
This repository is a sample of the course work from the CMPUT 412 course offered during the Winter of 2019 at the University of Alberta.

> "This course is an experimental inquisition of the fascinating field of mobile robotics. Students will specifically be exposed to basic concepts, models, and algorithms in autonomous navigation of a mobile robot operating in indoor environments. The instruction of the course will be through the Robot Operating System (ROS), which provides libraries and tools to help software developers quickly create robot applications. A mobile robot vehicle will be provided to the students, to conduct real experiments. It is equipped with a range sensor as well as a camera for the robot to perceive its environment and make navigational decisions. The course will be developed incrementally, through demos and competitions. The robot tasks that define the demos and competitions will begin with simple motion control and evolve toward a comprehensive term project involving  robot mapping, localization, path planning, object detection and homing."
([Source](https://www.ualberta.ca/computing-science/undergraduate-studies/course-directory/courses/experimental-mobile-robotics))

More information on the course can be found [here](https://www.ualberta.ca/computing-science/undergraduate-studies/course-directory/courses/experimental-mobile-robotics)

[Here is a sample of some of some of the course work](https://www.youtube.com/watch?v=_7_xZwPJDx8) and [here is the code for that sample](https://github.com/bofrim/CMPUT_412/tree/master/demo7).

## Purpose of This Document
This document is intended to serve as a springboard for you to start your journey with mobile robotics. We will provide you with a few backround concepts, some links to useful resources, and a description of how to get our code up and running. The intended audience of this document is anyone with a [passion for robotics and the willingness to learn](https://github.com/kiloreux/awesome-robotics). Aside from during the description of how this package is used, we do not assume that the reader has any backround in robotics concepts or really any other special technical background. When we describe how to setup and use this package we do assume that you have access to a [TurtleBot](https://www.turtlebot.com), you have a laptop with a copy of [Ubuntu 16.04 LTS](http://releases.ubuntu.com/16.04/) installed, and you are [familiar enough](https://help.ubuntu.com/community/UsingTheTerminal) with the Ubuntu operating system to navigate and install software. Let's get started!

## Backgound
Mobile Robotics is an [exciting field](https://www.nasa.gov/mission_pages/msl/index.html) with many [high value applicaitons](https://www.youtube.com/watch?v=zMTCMhy_MP4). We will provide a brief overview of some of the fundamental concepts used in the field of mobile robotics in order to get you up to speed.
#### ROS
> "The Robot Operating System (ROS) is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms."
([Source](http://www.ros.org/about-ros/))

ROS is a fantastic platform to aid in the development of robotics. It provides a [ton of great libraries](https://index.ros.org/packages/) that can be used right now to make some really cool robots without the overhead of doing absolutely everything yourself. ROS suppoorts development in Python and C++ so if you know either of those great languages, you can get started right here right now. ROS is open-source and has a wonderful support community full of helpful people who are willing to answer your quesitons in the [forums](https://answers.ros.org/questions/).

#### TurtleBot
The [TurtleBot](https://www.turtlebot.com) is a robot kit that provides the user with all of the components they need to start developing robots. It's main component is a base platform with a few wheels and a variety of sensors. You can find [tutorials](http://wiki.ros.org/Robots/TurtleBot#Robots.2BAC8-TurtleBot.2BAC8-kinetic.TurtleBot2) for getting started with the TurtleBot and quickly begin running demos or developing your own programs.
#### Mobile Robotic Concepts
[Mobile robotics](https://www.robotshop.com/community/tutorials/show/basics-what-types-of-mobile-robots-are-there) is a diverse field with a variety of cool, useful, and fun applications. Mobile robots usually consist of some form of robots base. The base will have some way to move around an environment. There are a variety of approaches that can be used for robot locomotion it could be wheels, tracks, legs, propellers, wings, rocket thrusters, or [something else](https://www.youtube.com/watch?v=divLsTtA5vk). The mobile robot also needs a way to derive feedback from it environnt. It may use a camera, light sensor, bump sensor, laser scanner, wisker sensor, sonar, lidar, some other kind of environmental feedback sensor, or it may employ a combination of a few different sensors. Sensor data can be used to help the robot navigate its surroundings or perform tasks.

##### Line Following
A [really cool but relatively simple demo](https://answers.ros.org/question/58443/line-following-in-ros/) for a mobile robot is to draw a line on the ground and then create a robot that will follow that line. Line following robots are a good way to start to learn some basic computer vision concepts as well as to learn how to write [simple control systems](https://www.youtube.com/watch?v=UR0hOmjaHp0).
##### Computer Vision
One of the main ways a robot can take in information from it environment is through some kind of camera. Computer vision processing can be applied to the input from a camera to help a robot determine what is happening around it. There are tons of great pre-made libraries allow you to use computer vision techniques withouth the need to know exactly how the algorithms work (even though they are pretty cool and you should [look into it](https://github.com/AGV-IIT-KGP/awesome-computer-vision#university-courses-and-moocs) if you get the chance).

One of the most popular computer vision libraries out there is [OpenCV](https://opencv.org). Using OpenCV you can:
* [Change and manipulate color spaces](https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_colorspaces/py_colorspaces.html)
* [Create threshold masks](https://docs.opencv.org/3.4/d7/d4d/tutorial_py_thresholding.html)
* [Count objects and detect shapes](https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_contours/py_contours_begin/py_contours_begin.html)
* [And so much more!](https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_tutorials.html)
##### Localization and Mapping
When your robot moves around, it is handy if it knows where it is. The concepts of localization and mapping are used to give robots the context of where it is in it's environment. Mapping is the process of collecting data about the surroundings of a robot and using it to build up a representation of the robot's environment in a format that is easy for the robot to understand. Localization involves using a robot's sensors to [statistically determine where to robot is](http://www.cim.mcgill.ca/~yiannis/particletutorial.pdf) relative to a map. Together localization and mapping can be used to help a robot navigate safely through it'senvironment.

A very useful ROS package to help with localization and mapping is [AMCL](http://wiki.ros.org/amcl). AMCL can be used to find and track a robot's location within a prebuilt map. This is very useful for navigation with a TurtleBot.
##### Other Concepts You Should Look Into
* Coordinate frames
TODO

## [CMPUT 412](https://www.youtube.com/watch?v=dQw4w9WgXcQ) Competitions
CMPUT 412's competitions were designed to introduce new challenges while building upon the last competition's implementation. 

#### Competition 1: Pursuit and Evasion
"Pursuit and Evasion" is the first competition of CMPUT 412. The main focus of this competition is for participants to get comfortable with the turtlebot's movement and working with the Asus Xtion Pro RGB-D camera. More information on the competition and running the code can be found in the /comp1/Competition1Report.pdf document in this repo.

#### Competition 2: Run, Robot Run
"Run, Robot Run" builds on top of Competition 1 by introducing a race-track shaped loop and object recognition tasks. Participants must use image recongition libraries such as OpenCV to follow the white line, stop at red lines, and complete the object recognition tasks located throughout the course. More information on Competition 2 can be found in the /comp2/Competition2Report.pdf document in this repo. 

#### Competition 3: Parking with GMapping, AMCL, and ARTags.
Competition 3 introduces the ROS navigation stack, recognizing AR-Tags, and create/using a map a file of the turtlebot's environment. The competition builds on top of the course used in Competition 2 by introducing a fourth location that is indicated by a 'off-ramp' in the course. Participants must use a map file to navigate through the new location, scan for objects, and park at specific parking spots. More information on Competition3 can be found in the /comp3/Competition3Report.pd document in this repo. 

#### Competition 4: Box-Pushing & Parking with the ROS Navigation stack
"Box-Pushing & Parking with the ROS Navigation stack" builds upon all previous topics covered in the course by introducing a new task, box pushing. Compeition 4 uses the same course as Competition 3 however the previous Location 4 parking task is replaced with pushing an AR-Tag covered box from one parking spot to another. No new topics are introduced, however participants must use their knowledge developed in the previous three competitions to identify, approach, and push the box. 

#### Competition 5: Game Strategy
Competition 4 is then followed by Competition 5, the final competition of the course. Competition 5 uses the same course as competition 4 but no mandatory tasks and a brand new marking scheme. Participants must evaluate the marking scheme and create a strategy to obtain the most amount of points in their 10 min attempt.
  
## Repository Structure
The structure of this repository acts as the contents of the src folder of the ROS catkin workpace. Each demo and competition is an individual catkin package, which allows for easy setup and support for future competitions. 

This repository is the src folder of the catkin workspace with each demo and competition as a seperate ROS catkin package. This layout allows for easy setup and updating the repo. 
  
## Getting Started
Installing and running the projects in this repo assume that you are comfortable with ROS and catkin. Consider consulting the ROS [official tutorials](http://wiki.ros.org/ROS/StartGuide) if you are unfamiliar with ROS or catkin. 
  
### Hardware Requirements
The demos and competitions in this repo assume that the user has access to:
- Turtlebot 2
- Asus Xtion Pro RGB-D Camera
- USB Camera
- Logitech Controller
- Laptop with 4 USB 2.0/3.0 Ports. (Warning: a USB hub may be used however we have found restrictions when using both the RGB-D camera and USB-cam on the same hub)

### Environment
This repo was developed on ROS Kinetic Kane on Ubuntu 16.04. ROS Kinetic is suppored on [multiple OS's](http://wiki.ros.org/kinetic/Installation), however we recommend Ubuntu 16.04 for the best results. The ROS Kinetic Kane installation for Ubuntu 16.04 can be found [here](http://wiki.ros.org/kinetic/Installation/Ubuntu)

### Setting up a Catkin Workspace
After ROS is properly installed a catkin workspace needs to be created for the demos and competitions found in this repo. Consult the official ROS documentation for setting up a [new catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace). 

### Installing this Repo
The structure of this project allows for easy installation and setup.

1. Navigate to the src folder of your catkin workspace
`cd {catkin_workspace}/src/`
2. Clone this repo 
`git clone https://github.com/bofrim/CMPUT_412.git`
3. Navigate to the root directory for your catkin workspace and build all the demos and competitions. 
`cd ..`
`catkin_make`


 MORE TODO
