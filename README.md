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
Use the following command
```
ros2 launch vbm_project_env simulation.launch.py
```

### Launching denoise pkg
In a new terminal, source the workspace and use the following command
```
ros2 run denoise denoise_pointcloud
```

### Launching major plane pkg
In a new terminal, source the workspace and use the following command
```
ros2 run major_plane_detection plane_detection
```


