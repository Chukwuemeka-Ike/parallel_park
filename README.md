# Autonomous DeepRacer Project for Robotics 1
This is an explanation of the dependencies of the parallel_park node used in
the Autonomous DeepRacer Project.

### Collaborators: John Parent and Chukwuemeka Ike

--------------------------------------------------------------------------------
--------------------------------------------------------------------------------

## cv_camera and image_proc

In order to standardize the image stream from the camera, the cv_camera and
image_proc standard packages were installed from the standard ROS repository.
The following depends on the proper setup and calibration of the camera.


## AprilTags 

The functionality of the parallel parking node is highly dependent on the
AprilTag ROS package available at:

"https://github.com/AprilRobotics/apriltag_ros.git"

The package has to be set up according to the instructions available at the
above link, with the configuration files. Once the package is properly set up,
the instructions available at:

"http://wiki.ros.org/apriltag_ros/Tutorials/Detection%20in%20a%20video%20stream"

are important in setting up the robot to continuously detect tags in a video
stream. To detect the tags used by this project, the user can simply replace
the 'settings' and 'tags' files in the 'config' folder with those of the same name
in the 'Txt Files' folder included with this submission.

Once AprilTags is setup as instructed, the user has to source the workspace
every time it will be used.

## parallel_park 

With apriltag's continuous node running in the background, the user simply needs
to build the parallel_park package using: "catkin_make" from the workspace
containing it. This will build the package and allow the user run:
"source ./devel/setup.bash" from the same workspace.

Finally, the "rosrun parallel_park parallel_park" command starts the
parallel_park node which executes the parallel parking functionality provided
the required conditions are at play.
