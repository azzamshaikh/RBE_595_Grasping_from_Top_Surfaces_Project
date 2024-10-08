# RBE 595 Group Project
This repo contains a ROS project developed for the RBE 550 group project.

## Getting the repo
Ensure that you have git access setup with your laptop.

If you are using HTTPS:
```
git clone https://github.com/azzamshaikh/RBE-595-Group-Project.git
```

If you are using SSH:
```
git clone git@github.com:azzamshaikh/RBE-595-Group-Project.git
```

## Run Instructions

Go to the `ros2_ws` folder
```
cd ros2_ws
```

Build the workspace
```
colcon build && source install/setup.bash
```

### Launching the simulation
Use the following command to launch the sim with 1 object
```
ros2 launch vbm_project_env simulation.launch.py
```

Use the following command to launch the sim with 3 objects
```
ros2 launch vbm_project_env simulation_multiple_objects.launch.py
```


