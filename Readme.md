


## Installation and usage

Dear Sevensense Team. I have used the humble branch for create3_sim, since I am using Ubuntu 22.04.3. Make sure to also run humble to make this project work.

In case you are using miniconda and ros can't find GLIBCXX_3.4.30, link the libstdc++ from usr/lib to the miniconda lib folder, i.e.
```ln -sf /usr/lib/x86_64-linux-gnu/libstdc++.so.6 /home/patrick/miniconda3/lib/libstdc++.so.6```

Install irobot msgs with:
```sudo apt-get install ros-humble-irobot-create-msgs```

I created a Makefile to run the commands comfortably. You can run the following commands to get the project running:
```make build-all```, to colcon build the packages.
```make spawn```, which spawns Rviz, Gazebo and a first robot.
```make spawn-second```, which adds another robot to the scene.
```make launch-alpha1```, which will run a controller for the first robot.
```make launch-alpha2```, which will run a controller for the second robot.

The last two commands can also be run at different times.

## Collision avoidance
Since both robots are running without communicating between each other, we assume that they have the capability of identifying the other's position when they get close enoguh. The `COLLISION_RADIUS` parameter defines the horizon where both robots can see each other, and take corresponding actions. For this, they listen to each other's odometry topics, since they have no sensors to measure the distance to the other robot. 

When they get close enough, they will stop and replan with a random path that does not intersect with each other's safe zone. This safe zone is defined as a half plane that passes through both robots, normal to the line that connects them. 

![Example](https://raw.githubusercontent.com/PatrissTV/sevensense-interview-project/main/recordings/collision.gif)

## Video
Here is a recording of the mission. The robots move to a random position and cover the map. They will stop when they get close enough to each other, and replan their path to avoid a collision.

<video src='https://raw.githubusercontent.com/PatrissTV/sevensense-interview-project/main/recordings/multi_agent_recording_seed_191.mp4' width=180/>

You can also download the video <a href="https://raw.githubusercontent.com/PatrissTV/sevensense-interview-project/main/recordings/multi_agent_recording_seed_191.mp4"> here </a>.


## Acknowledgements
Thank you for this opportunity. I am looking forward to hearing from you soon!