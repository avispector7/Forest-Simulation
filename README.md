# Forest-Environment

These are the resources for the forest environment framework.

## Integration

### Requirements
Unity Hub, Unity 2020
<br> ROS (this repository uses ROS Noetic)
<br> rospy

### Step 1: Clone the Repository

```
git clone https://github.com/avispector7/Forest-Environment.git
```

### Step 2: Install ARL Framework

Go to [ARL-Unity-ROS GitLab](https://gitlab.sitcore.net/arl/robotics-simulation/arl-unity-ros) and follow the instructions for Building from Source. Make sure the installation working properly.

### Step 3: Download ARL Unity Project

Clone the [ARL-Unity-Robotics](https://gitlab.sitcore.net/arl/robotics-simulation/arl-unity-robotics) Unity project into your Unity projects folder:

```
cd /path/to/Unity/projects
git clone https://gitlab.sitcore.net/arl/robotics-simulation/arl-unity-robotics
git lfs pull
```

Open the Unity Hub, click Open, and open the arl-unity-robotics project.

### [Optional] Step 4: Add Environment Project to Unity

Move the Environment2020 folder from this repository to your Unity projects folder. Open it from the Unity Hub the same way as above.

### Step 5: Add Environment Configuration Files to ARL Framework

Move the `environment` folder from this repository to `/path/to/arl-unity-ros/src/arl-unity-ros/environments`.

Move the `environment_with_husky.launch` file from this repository to `/path/to/arl-unity-ros/src/arl-unity-ros/arl_unity_ros_ground/launch`.

Build the arl-unity-ros workspace:

```
cd /path/to/arl-unity-ros
catkin build
```

### Step 6: Launch the Environment

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

In the script, in the `main()` method, under `# controls`, add commands to drive (forward/backward) and turn the Husky robot.

Make sure the environment simulation is running and run the script:

```
python /path/to/Forest-Environment/controller.py
```

## Editing the Environment

If you would like to make changes to the environment, open the Environment2020 project in Unity to make edits.

### Adding Your Updates

To add your changes to the ARL framework:

Delete the `AssetBundles` folder in `path/to/Unity/projects/Environment2020/Assets`.
<br> In the Unity project window, click on the file `Scene.unity`. On the lower right side of the inspector window, under "Asset Labels", click on the dropdown next to "AssetBundle" and click "New...". Type "environment" and press enter.
<br> In the Unity dropdown menu, click "Assets -> Build AssetBundles". A new `AssetBundles` folder should appear in the `Environment2020/Assets` folder where you previously deleted it.

In the `Environment2020/Assets/AssetBundles` folder, copy the file `environment`.
<br> In `/path/to/arl-unity-ros/src/arl-unity-ros/environments/environment/unity`, delete the `environment` file and replace it with the one you copied.

Build the arl-unit-ros workspace.

```
cd /path/to/arl-unity-ros
catkin build
```

## Common Error

When launching the environment, if at any point there is an error in the terminal in red that says `bind failed`, an error has occured when loading the environment.
<br> Stop the terminal process by pressing ctrl+C. Stop the Unity simulator by pressing the play button.
<br> Restart from Step 6 above.
<br> Repeat this if the error occurs again.
