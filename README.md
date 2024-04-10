# marine_robots_project
Codebase for the final project of DTU's [Autonomous Marine Robotics course](https://kurser.dtu.dk/course/34763)


# installation
From the root of the project, run the following commands:
1. build the docker images
```bash
docker-compose build
```
2. launch the 'dev' container
```bash
docker-compose up
```
3. open a new terminal and connect to the 'dev' container
```bash
docker exec -it final_project_dev_1 bash
```
4. Install the dependencies of the base ros packages
```bash
cd ros_ws
rosdep install --from-paths src --ignore-src -r -y
```
5. Try to build the ros packages
```bash
catkin build
```