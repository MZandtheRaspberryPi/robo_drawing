# robo_drawing

A general overview is presented here. First, pull the docker image. This contains ros2, pybullet. Then we will run the docker image letting it access X11 ports so as to bring up graphical user interfaces (GUIs) and letting it access the folder on our computer where we have our own code. Inside of the docker container, we will build our own code (should be fast, python) and then run it. To use the real robot we will download a special docker image (built for ARM, doesn't include moveit2 as it's too heavy) and then zip it and copy it to the raspberrypi on the robot. Then we will connect to the robot and run everything from there. Students in the past have had networking trouble related to MAC/Windows machines and therefore we avoid this issue by running everything on the robot.

```
cd $HOME
git clone https://github.com/MZandtheRaspberryPi/robo_drawing.git
cd robo_drawing
docker build -f docker/Dockerfile -t robodraw --progress plain .
docker run -it --rm -v $HOME/robo_drawing:/home/developer/ros_ws/src/robo_drawing --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    robodraw
cd ros_ws
colcon build --symlink-install --parallel-workers 4 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Now we bring up the simulation for the first time. This should produce a window similar to the first screenshot in this section. 
```
source install/setup.bash
ros2 launch ros_sim bringup_sim.launch.py robot_name:=franka_panda
```

To run the arm controller
```
ros2 run ros_arm_controller robo_controller --ros-args -p robot_name:=franka_panda
```

## Running on the mycobot arm

### copy your code over

Connect to the robot's WIFI. The WLAN should be named after the robot ID which is on a label on the base of the robot. Double check you are connected to the right robot--if you move a robot from somebody else's group that may surprise them and cause some damage to the robot or people.  SSID “mycobot_23”, password: "autonomy"
Robot ip address: 10.42.0.1, user: er, password: elephant (or Elephant) —“ssh er@10.42.0.1”

Compress and copy the folder over
```
cd ~
tar -czf robo_drawing.tar.gz robo_drawing
scp robo_drawing.tar.gz er@10.42.0.1:/home/er
```

Unzip the folder
```
ssh er@10.42.0.1
tar -mxzf robo_drawing.tar.gz
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
docker run --rm -it --network host --device /dev/ttyAMA0 -v /home/er/robo_drawing:/home/developer/ros_ws/src/robo_drawing -v /dev/shm:/dev/shm mzandtheraspberrypi/ros_sim:2024-10-24
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
ros2 run ros_arm_controller robo_controller --ros-args -p use_sim_time:=false -p robot_name:=mycobot_280
```


### If the image hasn't yet been built on the cobot:
Compress the entire folder so we can copy it over to the robot and build the docker image there...But, don't do it on the arm's raspberry pi, do it on a Raspberry Pi 4 or higher as the arm's raspberry pi isn't strong enough to build the image.

```
tar -czf /home/$USER/robo_drawing.tar.gz /home/$USER/robo_drawing
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
scp /home/$USER/robo_drawing.tar.gz er@192.168.1.113:/home/er
```

SSH via new connection with ip address (from angry ip scanner for example, will depend on network you connect to)
```
ssh er@192.168.1.113
tar -xzf robo_drawing.tar.gz
docker build -f docker/Dockerfile -t robo_drawing .
docker save robo_drawing:latest | gzip > robo_drawing_latest.tar.gz
```

copy docker image back out
```
scp er@192.168.1.113:/home/er/robo_drawing_latest.tar.gz .
```


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