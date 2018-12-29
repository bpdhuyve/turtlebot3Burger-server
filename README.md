# roboticsProject
## Project Robotics group 1: Piglet the MNM server

# Useful links
* Link to Demo Video, with SLAM and Navigation also displayed: https://youtu.be/zOzazwX_Oy0
* Slideshow with additional diagrams and photos clarifying the robots' functions: https://drive.google.com/open?id=1fPPpLrwimzrNosi9Ms0iaRYUDI5d-2mJ8Hu3Rw3Tv6Y

# Usage instructions
* On the raspberry pi, run the command: roslaunch piglet `bringup_pi.launch`. This will launch all the nodes which need to run on the raspberry pi.
* On the master device, run the command: roslaunch piglet `piglet.launch`. This will launch all the nodes which need to run on the master device, except for `find_people.py`
* When you want to start serving MNMs, launch `find_people.py` to start. Enjoy!

# Documentation:
The documentation for each major sub-component can be found under `doc/`. 
In this folder you pick whatever package you want to review and then proceed to open the `index.html` file inside its corresponding directory. This should open a browser window with the generated Epydoc documentation.

# List of packages
* OPEN_CR folder: Contains alternative firmware for the sonar to work on the openCR board.
## piglet
* `find_peoply.py`: Top level node that decides the movement of the robot. (master)
* `math_functions.py`: Contains some methods for mathematical calculations.
* `simulation_publishers`: Folder with code necessary for simulation for the sake of testing and debugging.

## sensors
* `ir_pub`: Publishes booleans that determine whether one of the IR sensors is triggered. The value of each sensor is published to its respective topic. (raspberry pi)
* `sonar.py`: Publishes the values of the sonar sensors to the sonar topic. (raspberry pi)
* `pointcloud_pub.py`: Creates Points based on the values of the sensors, and publishes these as pointclouds to the navigation stack. (raspberry pi)

## face_rec
* `FaceRecognizer.py`: Class with face detection, recognition and comparison funcionilty
* `face_recognition_publisher.py`: Sends images from PI to master (raspberry pi)
* `face_recognition_subscriber.py`: Receives faces and sends id and coordinates to master (master)

## human_robot_interaction
* `send_audio.py`: Continuously listens to speech. (raspberry pi)
* `key_phrase_test.py`: Will receive audio and detect if keyphrase _hi Piglet_ is said, publishes boolean when keyphrase is detected. (raspberry pi)
* `kws_test.py`: Will listen to audio and if keyphrase is detected, interpret audio as commands. (raspberry pi)
* `action_handler.py`: Will map detected command phrases to the actual command, and respond with speech. (raspberry pi)

# Hardware Description
## Sensors:
* LiDaR: Used to detect obstacles taller than the robot.
* IR sensors: 4 sensors at each corner of the robot, for edge detection. 1 sensor to detect if an M&M has been taken. 1 sensor to detect obstacles behind the robot. See `sensors/src`.
* Sonars: 2 at the front of the robot to detect low obstacles. Working but disabled for demo because the results are slightly worse when using them, because of sporadic false detection of objects. See `sensors/src`.

## Speaker:
Used for communicating with the people on the table (e.g. saying that someone has been recognized, or that someone has taken an M&M). See `human_robot_interaction`.

## LEDs:
Used as decorative 'eyes' of the robot.

## Camera:
For recognizing people. People that are recognized are put in a list, and the algorithm can also distinguish between people. See `face_rec\src`.


