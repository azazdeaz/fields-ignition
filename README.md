# fields-ignition 
### *random crop field generator for Ignition Gazebo*

![](images/demo.gif)

:construction: work in progress

 - [What's this?](#whats-this)
 - [Notes](#notes)
 - [Run in a catkin workspace](#run-in-a-catkin-workspace)
 - [Run with Docker](#run-with-docker)

## What's this?
I was wondering how complicated would it be to generate random test environments for agricultural robots. Turned out with great tools like the Blender python API, and Ignition Gazebo it can be a fun and straightforward process :tada:

This repo contains:
 - [Blender script](fields_ignition/blender/tomato_gen.py) that generates individual tomato crops
 - [Jupyter notebook](fields_ignition/scripts/tomato_gen.ipynb) for building the SDF models and the world description
 - [ROS node](fields_ignition/scripts/ground_truth.py) that visualizes the ground truth data in RViz
 - [Dockerfile](Dockerfile) for running the simulations
 - [Example worlds](fields_ignition/generated_examples) generated with this tool

## Notes
 - The generated results are reproducible by specifying the seed value
 - Each model has a markers.json listing the position of the fruits relative to the crop
 - Also, the world folder has a markers.json listing all the plant and fruit positions relative to the world origin

![random tomato crops in Blender](images/tomatoes.gif)

## Run in a catkin workspace
*requires ROS Noetic and Ignition Dome*

### Install
 - `git clone https://github.com/azazdeaz/fields-ignition.git` in the src folder
 - `rosdep install --from-paths src --ignore-src -r -y` in the workspace folder
 -  and build with `catkin_make`

### Run the simulation
 - run an example world: `roslaunch fields_ignition field.launch world_dir:=$(rospack find fields_ignition)/generated_examples/tomato_field`
 - visualize the ground truth data with rviz: `roslaunch fields_ignition ground_truth.launch world_dir:=$(rospack find fields_ignition)/generated_examples/tomato_field`
  
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
