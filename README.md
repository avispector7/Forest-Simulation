# Simulated Forest Environment and Robot Control Framework
Resources for simulating a forest environment with a Clearpath Husky robot and controlling the robot. Uses ARL Unity ROS framework.

[<img width="1440" alt="Demo Video Thumbnail" src="https://user-images.githubusercontent.com/108345269/196803150-2af05a21-e98e-4e22-b22b-a0ceef765ee2.png">](https://www.youtube.com/watch?v=Y9XodP4n9v4)
[Demo Video](https://www.youtube.com/watch?v=Y9XodP4n9v4)

## Integration

### Requirements

Unity Hub, Unity 2020
<br> ROS (this repository uses ROS Noetic)
<br> rospy

### Step 1: Clone the Repository

```
git clone https://github.com/avispector7/Forest-Simulation.git
```

### Step 2: Install the ARL Framework

Go to [ARL-Unity-ROS GitLab](https://gitlab.sitcore.net/arl/robotics-simulation/arl-unity-ros) and follow the instructions for Building from Source. Make sure the installation working properly.

### Step 3: Download the ARL Unity Project
Clone the [ARL-Unity-Robotics](https://gitlab.sitcore.net/arl/robotics-simulation/arl-unity-robotics) Unity project into your Unity projects folder (wherever you store/would like to store your Unity project files):

```
cd /path/to/Unity-projects
git clone https://gitlab.sitcore.net/arl/robotics-simulation/arl-unity-robotics
cd arl-unity-robotics
git lfs pull
```

Open the Unity Hub, click Open, and open the arl-unity-robotics project.

### Step 4: Add the Environment Configuration Files to the ARL Framework

Move the `environment` folder from this repository to `/path/to/arl-unity-ros/src/arl-unity-ros/environments`.

Move the `environment_with_husky.launch` file from this repository to `/path/to/arl-unity-ros/src/arl-unity-ros/arl_unity_ros_ground/launch`.
<br> ***Important:*** Change line 9 of this file to the path to the arl-unity-ros workspace on your computer.

Build the arl-unity-ros workspace:

```
cd /path/to/arl-unity-ros
catkin build
```

### Step 5: Launch the Environment

Open the ARL Unity Project.
<br> Under `Assets/SimulationCore` open the `Simulation.unity` scene.
<br> Click play at the top of the window and wait for it to load.

Run the following commands:

```
cd /path/to/arl-unity-ros
source devel/setup.bash
roslaunch arl_unity_ros_ground environment_with_husky.launch
```

If you want to open an RViz window with the camera image and lidar data, replace the last command with

```
roslaunch arl_unity_ros_ground environment_with_husky.launch rviz:=true
```

## Robot Control

To control the Husky robot using the included `controller.py` script:

In the script, in the `main()` method, under `# controls`, add commands to drive (forward/backward) and turn the Husky robot. Instructions for these commands, as well as a few sample movement commands, are in the file.

Make sure the environment simulation is running and run the script:

```
python /path/to/Forest-Simulation/controller.py
```

## Editing the Environment

Download the [Forest-Environment](https://github.com/avispector7/Forest-Environment) project into your Unity projects folder:

```
cd /path/to/Unity-projects
git clone https://github.com/avispector7/Forest-Environment.git
cd Forest-Environment
git lfs pull
```

Open the Unity Hub, click Open, and open the Forest-Environment project.
<br> Make whatever changes to the environment you would like. Don't forget to save.

A list of the Unity assets used in this project are on the Forest-Simulation GitHub.

### Adding Your Updates

To add your changes to the ARL framework:

Delete the `AssetBundles` folder in `path/to/Unity-projects/Forest-Environment/Assets`.

In the Unity project window, click on the file `Scene.unity`.
<br> On the lower right side of the inspector window, under "Asset Labels", click on the dropdown next to "AssetBundle" and click "New...".
<br> Type "environment" and press enter.

In the Unity dropdown menu, click "Assets -> Build AssetBundles".
<br> A new `AssetBundles` folder should appear in the `Forest-Environment/Assets` folder where you previously deleted it.

In the `Forest-Environment/Assets/AssetBundles` folder, copy the file `environment`.
<br> In `/path/to/arl-unity-ros/src/arl-unity-ros/environments/environment/unity`, delete the `environment` file and replace it with the one you copied.

Build the arl-unit-ros workspace:

```
cd /path/to/arl-unity-ros
catkin build
```

## Common Error

When launching the environment, if at any point there is an error in the terminal in red that says `bind failed`, an error has occured when loading the environment.
<br> Stop the terminal process by pressing ctrl+C. Stop the Unity simulator by pressing the play button.
<br> Restart from Step 5 above.
<br> Repeat this if the error occurs again.
