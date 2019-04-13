# CMPUT 412 Winter 2019 
This repository is a sample of the course work from the CMPUT 412 course offered during the Winter of 2019 at the University of Alberta.

> "This course is an experimental inquisition of the fascinating field of mobile robotics. Students will specifically be exposed to basic concepts, models, and algorithms in autonomous navigation of a mobile robot operating in indoor environments. The instruction of the course will be through the Robot Operating System (ROS), which provides libraries and tools to help software developers quickly create robot applications. A mobile robot vehicle will be provided to the students, to conduct real experiments. It is equipped with a range sensor as well as a camera for the robot to perceive its environment and make navigational decisions. The course will be developed incrementally, through demos and competitions. The robot tasks that define the demos and competitions will begin with simple motion control and evolve toward a comprehensive term project involving  robot mapping, localization, path planning, object detection and homing."
([Source](https://www.ualberta.ca/computing-science/undergraduate-studies/course-directory/courses/experimental-mobile-robotics))

More information on the course can be found [here](https://www.ualberta.ca/computing-science/undergraduate-studies/course-directory/courses/experimental-mobile-robotics)

[Here is a sample of some of some of the course work](https://www.youtube.com/watch?v=_7_xZwPJDx8) and [here is the code for that sample](https://github.com/bofrim/CMPUT_412/tree/master/demo7).

## Purpose of This Document
This document is intended to serve as a springboard for you to start your journey with mobile robotics. We will provide you with a few backround concepts, some links to useful resources, and a description of how to get our code up and running. The intended audience of this document is anyone with a [passion for robotics and the willingness to learn](https://github.com/kiloreux/awesome-robotics). Aside from the description of how this package is used, we do not assume that the reader has any backround in robotics concepts or really any other special technical background. When we describe how to setup and use this package we do assume that the reader has access to a [TurtleBot](https://www.turtlebot.com), your have a laptop with a copy of [Ubuntu 16.04 LTS](http://releases.ubuntu.com/16.04/) installed, and you are [familiar enough](https://help.ubuntu.com/community/UsingTheTerminal) with the Ubuntu operating system to navigate and install software. Let's get started!

## Backgound
Mobile Robotics is an [exciting field](https://www.nasa.gov/mission_pages/msl/index.html) with many [high value applicaitons](https://www.youtube.com/watch?v=zMTCMhy_MP4). We will provide a brief overview of some of the fundamental concepts used in the field of mobile robotics in order to get you up to speed.
#### ROS
> "The Robot Operating System (ROS) is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms."
([Source](http://www.ros.org/about-ros/))
ROS is a fantastic platform to aid in the development of robotics. It provides a [ton of great libraries](https://index.ros.org/packages/) that can be used right now to make some really cool robots without the overhead. ROS suppoorts development in Python and C++ so if you know either of those great languages, you can get started right here right now. ROS is open-source and has a wonderful support community full of helpful people who are willing to answer your quesitons in the [forums](https://answers.ros.org/questions/).
#### TurtleBot
The [TurtleBot](https://www.turtlebot.com) is a robot kit that supplys the user with all of the components they need to start developing robots. It's main component is a base with a variety of sensors and some wheels. You can find [tutorials](http://wiki.ros.org/Robots/TurtleBot#Robots.2BAC8-TurtleBot.2BAC8-kinetic.TurtleBot2) for getting started with the TurtleBot and quickly begin running demos or developing your own programs.
#### Mobile Robotic Concepts
[Mobile robotics](https://www.robotshop.com/community/tutorials/show/basics-what-types-of-mobile-robots-are-there) is a diverse field with a variety of cool, useful, and fun applications. Mobile robots usually consist of some form of robots base. The base will have some method movement whether it is, wheels, tracks, legs, or something else. The mobile robot also needs a way to dreive feedback from it environnt. It may use a camera, light sensor, bump sensor, laser scanner, wisker sensor, sonar, lidar, some other kind of environmental feedback sensor, or it may employ a combination of a few different sensors.

##### Line Following
A [really cool but relatively simple demo](https://answers.ros.org/question/58443/line-following-in-ros/) for a mobile robot is to draw a line on the ground and then create a robot that will follow that line. Line following robots are a good way to start to learn some basic computer vision concepts as well as how to write a simple control system.
##### Computer Vision
One of the main ways a robot can take in information from it environment is through some kind of camera. Computer vision concepts can be applied tor the input from a camera to help a robot determine what is happening around it. There are tons of great pre-made libraries allow you to use computer vision techniques withouth the need to know exactly how the algorithms work (even though they are pretty cool and you should [look into it](https://github.com/AGV-IIT-KGP/awesome-computer-vision#university-courses-and-moocs) if you get the chance).

One of the most popular computer vision libraries out there is [OpenCV](https://opencv.org). Using OpenCV you can:
* [Change and manipulate color spaces](https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_colorspaces/py_colorspaces.html)
* [Create threshold masks](https://docs.opencv.org/3.4/d7/d4d/tutorial_py_thresholding.html)
* [Count objects and detect shapes](https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_contours/py_contours_begin/py_contours_begin.html)
* [And so much more!](https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_tutorials.html)
##### Localization and Mapping
When your robot moves around, it is handy if it knows where it is. The concepts of localization and mapping are used to give robots the context of where it is in it's environment. Mapping is the process of collecting data about the surroundings of a robot and using it to build up a representation of the robot's environment in a format that is easy for the robot to understand. Localization involves using a robot's sensors to statistically determine where to robot is relative to a map. Together localization and mapping can be used to help a robot navigate safely through it'senvironment.

A very useful ros package to help with localization and mapping is [AMCL](http://wiki.ros.org/amcl). AMCL can be used to find and track a robot's location within a prebuilt map. This is very useful for navigation with a TurtleBot.
##### Other Concepts You Should Look Into
* Coordinate frames

## [CMPUT 412](https://www.youtube.com/watch?v=dQw4w9WgXcQ) Competitions
### Competition Structure
### Repository Structure

## Getting Started
### Environment
### Installing Requirements
### Setting up a Catkin Workspace
### Installing this Repo
