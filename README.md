# marine_robots_project
Codebase for the final project of DTU's [Autonomous Marine Robotics course](https://kurser.dtu.dk/course/34763)


# installation
From the root of the project, run the following commands:
1. build the docker images
```bash
docker compose build
```
2. launch the 'dev' container
```bash
docker compose up
```
you should now have a running container with the name 'final_project_dev_1'

3. open a new terminal and connect to the 'dev' container
```bash
docker exec -it final_project_dev_1 bash
```
4. Install the dependencies of the base ros packages
```bash
cd ros_ws
rosdep install --from-paths src --ignore-src -y
```
5. Try to build the ros packages
```bash
catkin build -p2 -j8 --mem-limit 50%
```
6. Add the sourcing of your workspace to your .bashrc
```bash
echo "source ${HOME}/ros_ws/devel/setup.bash" >> ~/.bashrc
```

# Launching the simulation
run the following command **inside** the 'dev' container
1. Enter the dev container
```bash
docker exec -it final_project_dev_1 bash
```
2. Launch the simulation
```bash
roslaunch bluerov2_gazebo start_pid_demo.launch
```

# Additional information
- Some useful aliases are defined in the `utils/aliases.sh` file. You can source it by running `source utils/aliases.sh` **outside** the container. **Important** inside that file, the `base_dir` variable should be set to the path of the project in the container.