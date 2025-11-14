# robo_drawing

```
docker run -it --rm -v $HOME/robo_drawing:/home/ros_ws/src/robo_drawing --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    robodraw
cd ros_ws
colcon build --symlink-install --parallel-workers 4 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Now we bring up the simulation for the first time. This should produce a window similar to the first screenshot in this section. 
```
source install/setup.bash
ros2 launch ros_sim bringup_sim.launch.py
```




A general overview is presented here, details and steps are in sub-headers. First, pull the docker image. This contains ros2, pybullet, and moveit2 with all the libraries needed for this lab and everything compiled for you (moveit2 takes like 30min to compile), you're welcome. Then we will run the docker image letting it access X11 ports so as to bring up graphical user interfaces (GUIs) and letting it access the folder on our computer where we have our own code. Inside of the docker container, we will build our own code (should be fast, python) and then run it. To use the real robot we will download a special docker image (built for ARM, doesn't include moveit2 as it's too heavy) and then zip it and copy it to the raspberrypi on the robot. Then we will connect to the robot and run everything from there. Students in the past have had networking trouble related to MAC/Windows machines and therefore we avoid this issue by running everything on the robot.

## get docker file

```
docker pull --platform linux/amd64 mzandtheraspberrypi/ros_sim:2025-10-24
```

## running examples on your laptop

```
docker run --rm --platform linux/amd64 -it -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY -e QT_X11_NO_MITSHM=1 --network host -v /home/student/ros_sim:/home/developer/ros_ws/src/ros_sim mzandtheraspberrypi/ros_sim:2025-10-24
sudo apt-get remove -y python3-matplotlib
cd ros_ws
colcon build --symlink-install --packages-up-to ros_sim ros_arm_controller ros_cobot_controller
source install/setup.bash
ros2 run ros_sim pybullet_ex
```

A command may be published with the below:
```
ros2 topic pub -1 /target_joint_states sensor_msgs/msg/JointState "{position: [1.5, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

You may open more terminals to allow introspection with:
```
docker exec -it $(docker ps -lq) /bin/bash
cd ros_ws
source install/setup.bash
ros2 topic list
```

To run the controller:

```
docker exec -it $(docker ps -lq) /bin/bash
cd ros_ws
source install/setup.bash
ros2 run ros_arm_controller robo_controller --ros-args -p use_sim_time:=true
```

To run Moveit2:
```
colcon build --symlink-install --packages-select mycobot_moveit2
ros2 launch mycobot_moveit2 demo.launch.py
```

## Running on the arm

### copy your code over

Connect to the robot's WIFI. The WLAN should be named after the robot ID which is on a label on the base of the robot. Double check you are connected to the right robot--if you move a robot from somebody else's group that may surprise them and cause some damage to the robot or people.  SSID “mycobot_23”, password: "autonomy"
Robot ip address: 10.42.0.1, user: er, password: elephant (or Elephant) —“ssh er@10.42.0.1”

Compress and copy the folder over
```
cd ~
tar -czf ros_sim.tar.gz ros_sim
scp ros_sim.tar.gz er@10.42.0.1:/home/er
```

Unzip the folder
```
ssh er@10.42.0.1
tar -mxzf ros_sim.tar.gz
```

### copy arm docker image to the cobot

This step only needs to be done once.

Ensure your computer is connected to the internet:

```
cd ~
docker pull --platform linux/arm64 mzandtheraspberrypi/ros_sim:2024-10-24
docker image save --platform linux/arm64 mzandtheraspberrypi/ros_sim:2024-10-24 | gzip > docker_ros_sim_arm.tar.gz
```

Now connect to the robot's WIFI.
```
scp /home/$USER/ros_sim.tar.gz er@10.42.0.1:/home/er
```
```
ssh er@10.42.0.1:/home/er
docker system prune -a
docker images # remove big images you see here
docker load --input docker_ros_sim_arm.tar.gz
rm docker_ros_sim_arm.tar.gz
```


### running examples on the robot

```
docker run --rm -it --network host --device /dev/ttyAMA0 -v /home/er/ros_sim:/home/developer/ros_ws/src/ros_sim -v /dev/shm:/dev/shm mzandtheraspberrypi/ros_sim:2024-10-24
sudo apt-get remove -y python3-matplotlib
cd ros_ws
colcon build --symlink-install
source install/setup.bash
ros2 run ros_cobot_controller cobot_controller --ros-args -p mycobot_speed:=30
```

```
docker exec -it $(docker ps -lq) /bin/bash
cd ros_ws
source install/setup.bash
ros2 run ros_arm_controller robo_controller --ros-args -p use_sim_time:=false
```


### If the image hasn't yet been built on the cobot:
Compress the entire folder so we can copy it over to the robot and build the docker image there...

```
tar -czf /home/$USER/ros_sim.tar.gz /home/$USER/ros_sim
```

Connect to the robot's WIFI via your Ubuntu wifi interface (top right of screen). The WLAN should be named after the robot ID which is on a label on the base of the robot. Double check you are connected to the right robot--if you move a robot from somebody else's group that may surprise them and cause some damage to the robot or people. Check the robot's IP address, and SCP the file into the robot.

Connect to robot WiFi—“mycobot_23”, password: autonomy
Robot ip address: 10.42.0.1, user: er, password: elephant (or Elephant) —“ssh er@10.42.0.1”

enable on screen keyboard
```
gsettings set org.gnome.desktop.a11y.applications screen-keyboard-enabled true
```

Connect HDMI and mouse to robot, switch wifi to one with internet using virtual keyboard (to build docker image).

Copy the folder over....
```
scp /home/$USER/ros_sim.tar.gz er@192.168.1.113:/home/er
```

SSH via new connection with ip address (from angry ip scanner for example, will depend on network you connect to)
```
ssh er@192.168.1.113
tar -xzf ros_sim.tar.gz
docker build -f docker/Dockerfile -t ros_sim .
docker save ros_sim:latest | gzip > ros_sim_latest.tar.gz
```

copy docker image back out
```
scp er@192.168.1.113:/home/er/ros_sim_latest.tar.gz .
```


## building dockerfile

Has to be built for ARM and for AMD. don't try and build it on a raspberry pi 3b+. Use 4 or higher more. Or build it on a jetson and then transfer it.

```
docker build -f docker/Dockerfile -t ros_sim --progress plain .
```

## citations

https://mathweb.ucsd.edu/~sbuss/ResearchWeb/ikmethods/SdlsPaper.pdf for overview including jacobian iterations, and damped least squares

## troubleshooting:

If you see this error, simply re-launch.
```
developer@student-AORUS-5-KB:~/ros_ws$ ros2 run ros_sim pybullet_ex 
pybullet build time: Sep 30 2025 11:26:09
[INFO] [1759315578.864487194] [ros_sim_node]: loading: /home/developer/ros_ws/install/ros_sim/share/ros_sim/resource/robots/mycobot_280_pi/mycobot_280_pi_mod.urdf
argv[0]=
startThreads creating 1 threads.
starting thread 0
started thread 0 
argc=3
argv[0] = --unused
argv[1] = 
argv[2] = --start_demo_name=Physics Server
ExampleBrowserThreadFunc started
X11 functions dynamically loaded using dlopen/dlsym OK!
X11 functions dynamically loaded using dlopen/dlsym OK!
MESA: error: Failed to query drm device.
libGL error: glx: failed to create dri3 screen
libGL error: failed to load driver: iris
libGL error: failed to open /dev/dri/card1: No such file or directory
libGL error: failed to load driver: iris
Creating context
Created GL 3.3 context
Direct GLX rendering context obtained
Making context current
GL_VENDOR=Mesa
GL_RENDERER=llvmpipe (LLVM 15.0.7, 256 bits)
GL_VERSION=4.5 (Core Profile) Mesa 23.2.1-1ubuntu3.1~22.04.3
GL_SHADING_LANGUAGE_VERSION=4.50
pthread_getconcurrency()=0
Version = 4.5 (Core Profile) Mesa 23.2.1-1ubuntu3.1~22.04.3
Vendor = Mesa
Renderer = llvmpipe (LLVM 15.0.7, 256 bits)
b3Printf: Selected demo: Physics Server
startThreads creating 1 threads.
starting thread 0
started thread 0 
MotionThreadFunc thread started
ven = Mesa
X Error of failed request:  BadValue (integer parameter out of range for operation)
  Major opcode of failed request:  130 (MIT-SHM)
  Minor opcode of failed request:  3 (X_ShmPutImage)
  Value in failed request:  0x400
  Serial number of failed request:  61
  Current serial number in output stream:  62
[ros2run]: Process exited with failure 1
```