# Localization
ROS2 Localization repository(Team Elite).

## Main Contributor
Nirsitha (https://git.hs-coburg.de/Nirsitha)

## Component Description
The Localization Component determines the precise position and orientation of the autonomous pupil shuttle using OptiTrack motion capture data. It provides real-time pose information to support navigationand path planning, ensuring safe and efficient operation.

## Table of Contents
- [Nodes](#nodes)
- [RQT_graph](#rqt_graph)
- [Installation](#installation)
- [Usage](#usage)
- [Testing](#testing)
- [License](#license)


## Nodes
### Node: `path and trajectory Planner`
#### Topics

| **Topic Name**            | **Input/Output**    | **Message Type**             | **Description** |
|---------------------------|---------------------|------------------------------|-----------------|
| `/pose_modelcars`         | **Input** (Subscribe) | `geometry_msgs/PoseStamped`  |Publishes the position and orientation data of multiple rigid bodies (e.g., cars). |
| `/team_elite_ego_pose`     | **Output** (Publish)  | `geometry_msgs/msg/pose`     |Publishes the position and orientation of a team elite rigid body |

## RQT_graph
![rosgraphall](https://github.com/user-attachments/assets/ec209f53-eff4-4038-85ee-050bdebcffc5)

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
ros2 run loc_node loc_package 
```

## Testing
### Unit Tests
To run the unit tests for this package, use the following command:

```bash
colcon test --packages-select loc_node
```

## License

This project is licensed under the **Apache 2.0 License** - see the [LICENSE](LICENSE) file for details.



