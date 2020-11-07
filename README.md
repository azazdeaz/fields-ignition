# fields-ignition 
### *random crop field generator for Ignition Gazebo*

[](images/demo.gif)

:construction: work in progress

 - [How it works?](#how-it-works)
 - [Notes](#notes)
 - [Install](#install)

## How it works?
 TODO

## Run in a catkin workspace
*requires ROS Noetic and Ignition Dome*

### Install
 - `git clone https://github.com/azazdeaz/fields-ignition.git` in the src folder
 - `rosdep install --from-paths src --ignore-src -r -y` in the workspace folder
 -  and build with `catkin_make`

### Run the simulation
 - run an example world: `roslaunch fields_ignition field.launch world_dir:=$(rospack find fields_ignition)/generated_examples/tomato_field`
 - visualize the ground truth data with rviz: `roslaunch fields_ignition ground_truth.launch world_dir:=$(rospack find fields_ignition)/generated_examples/tomato_field`
 - 
### Generate a new tomato field
 - start the notebook `rosrun fields_ignition tomato_gen_notebook`
 - modify the parameters in the second cell and run the notebook

 
## Run with Docker
```bash
# build the container
docker build -t ros-tomatoes .
```

```bash
# run the simulator with Docker:
xhost +

docker run -it \
     -e WORLD_DIR=/catkin_ws/src/fields_ignition/generated_examples/tomato_field \
     -e DISPLAY \
     -e QT_X11_NO_MITSHM=1 \
     -e XAUTHORITY=$XAUTH \
     -v "$XAUTH:$XAUTH" \
     -v "/tmp/.X11-unix:/tmp/.X11-unix" \
     -v "/etc/localtime:/etc/localtime:ro" \
     -v "/dev/input:/dev/input" \
     --network host \
     --rm \
     --privileged \
     --runtime=nvidia \
     --security-opt seccomp=unconfined \
     --name tomato_field \
     ros-tomatoes
```
> Note: to start a less resource heavy simulation change the `WORLD_DIR` env to `/catkin_ws/src/fields_ignition/generated_examples/tomato_field_mini` 
