# Localization
ROS2 Localization repository(Team Elite).

## Main Contributor
([@Nirsitha](https://git.hs-coburg.de/Nirsitha))

## Component Description
The Localization Component determines the precise position and orientation of the autonomous pupil shuttle using OptiTrack motion capture data. It provides real-time pose information to support navigation and path planning, ensuring safe and efficient operation.

The block diagram below illustrates the connected components involved in the Localization Component:

<img src="loccomp.jpg" alt="block_diagram" width="750">

## Table of Contents
- [Nodes](#nodes)
- [RQT_graph](#rqt_graph)
- [Installation](#installation)
- [Usage](#usage)
- [Testing](#testing)
- [License](#license)


## Nodes
### Node: `Localization`
#### Topics

| **Topic Name**            | **Input/Output**    | **Message Type**             | **Description** |
|---------------------------|---------------------|------------------------------|-----------------|
| `/pose_modelcars`         | **Input** (Subscribe) | `geometry_msgs/PoseStamped`  |Subscribe the position and orientation data of multiple rigid bodies from OptiTrack (e.g., cars). |
| `geometry_msgs/ego_pose`     | **Output** (Publish)  | `geometry_msgs/msg/Pose`     |Publishes the position and orientation of a team elite rigid body  to Path andTrajectory planner, longitudinal and lateral controller, communication|
| `/ego_twist`     | **Output** (Publish)  | `geometry_msgs/msg/Twist`     |Publishes the angular and linear velocity of a team elite rigid body to Path, Trajectory planner, longitudinal and lateral controller, communication|


## RQT_graph

<img src="rosgraphloc.png" alt="block_diagram" width="750">

## Installation
1. Clone the repository:
```bash
 git clone https://git.hs-coburg.de/The_ELITE/TheElite_Localization.git
```
2. Build the package:
```bash
 colcon build --packages-select loc_package
```
3. Source the workspace:
```bash
 source install/setup.bash
```

## Usage
### Launching the Nodes
To launch all of the nodes in lateral control package, run the following command:

```bash
ros2 run vehicle_localization loc_package 
```

## Testing
### Unit Tests
To run the unit tests for this package, use the following command:

```bash
colcon test --packages-select vehicle_localization
```

## License

This project is licensed under the **Apache 2.0 License** - see the [LICENSE](LICENSE) file for details.



