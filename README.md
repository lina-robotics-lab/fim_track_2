# fim_track_2

This is the ROS2 version of fim_track package. Testing on ROS2 dashing, Gazebo 9, Ubuntu 18.04.

---



### First Steps and Example Usage

**Prerequisite: on a Ubuntu 18.04/20.04 system, install ROS2 and Gazebo 9/11(following the instructions on https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup)**. 

Assume you have followed all the instructions on Turltebot3 emanual website, your current ROS workspace directory should be **turtlebot3_ws**. 

Clone fim_track_2 repository into turtlebot3_ws/src. 

Call `$colcon build --symlink-install` to build the package. 

And call `$source /turtlebot3_ws/install/setup.bash` to ensure the executables in fim_track_2 are visible to `ros2` command.

Then in a terminal, add the model files to `$GAZEBO_MODEL_PATH`

`$export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/turtlebot3_ws/src/fim_track_2/models`

Now we should be able to launch the pre-configured gazebo simulation via

`$ros2 launch fim_track_2 gazebo_simulation_launch.py`

Use `$ros2 topic list` to view the collection of published topics.

Then use

`$ros2 run fim_track_2 manual_teleop_key <robot_namespace>`

without the ".py" postfix to "manual_teleop_key" to control individual robots using keyboard.

### Special attention when running my_tb3_bringup for the first time on a Turtlebot

The specification file **burger.yaml** needs to be editted for the **my_tb3_bringup** script to work. Specifically, go to folder _/param_ and open **burger.yaml**, replace **MobileSensor3** with the hostname of the new robot. Here hostname can be found by typing **hostname** in the Linux terminal.
