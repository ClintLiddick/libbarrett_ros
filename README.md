# libbarrett_ros
libbarrett_ros provides a ros_control hardware interface for [Barrett Technology](http://www.barrett.com/)'s products. This library was developed by the [Personal Robotics Lab](https://personalrobotics.ri.cmu.edu/) at Carnegie Mellon University in collaboration with [Barrett Technology](http://www.barrett.com/).

> **WARNING:** libbarrett_ros is under heavy development and has not been fully tested. Please contact [herb-hardware@lists.andrew.cmu.edu](mailto:herb-hardware@lists.andrew.cmu.edu) if you are interested in using or contributing to this package.

## Dependencies
This package requires [ROS](http://www.ros.org/), [ros_control](http://wiki.ros.org/ros_control), and [libbarrett](https://github.com/personalrobotics/libbarrett) to be installed. We have tested libbarrett_ros using ROS Hydro on Ubuntu 12.04 and ROS Indigo on Ubuntu 14.04.

## Configuration
This package includes one ROS node, called `libbarrett_ros`, that communicates with Barrett hardware over one or more communication busses. Each communication bus is configured by a directory of libbarrett configuration files. A default set of configuration files are distributed with libbarrett and are installed to `/etc/barrett`. You may need to customize these files for your particular robot (e.g. change the home or gravity calibration configurations).

You can specify a list of `default.conf` configuration files in the `~configurations` ROS parameter, e.g.:

```yaml
configurations:
- /etc/barrett/default.conf
```

If your robot uses multiple CAN buses, then you must specify a list of configuration directories, e.g.:

```yaml
configurations:
- /etc/barrett/right_arm/default.conf
- /etc/barrett/left_arm/default.conf
```

## Usage
Once the necessary parameters have been set, you can start the controller manager:
```bash
rosrun libbarrett_ros libbarrett_ros
```

This will start a ros_control `ControllerManager` that is communicating with the hardware. If you are controlling a WAM, then you will be prompted to zero the WAM (if necessary) and press Shift + Activate on the pendant before the control loop starts. By default, until a controller is loaded, the arm will be applying zero torque to all joints. **Be careful: the brakes are disengaged and gravity compensation is disabled**, so you rest the arm in a stable configuration.

You will most likely want to load one or more ros_control controllers. See [the `ros_control` documentation](http://wiki.ros.org/ros_control/Tutorials/Loading%20and%20starting%20controllers%20through%20service%20calls) for more information.

## Examples
This package includes an example configuration for HERB, a bimanual mobile manipulator that has two Barrett WAMs. Each WAM is equipped with a Barrett force/torque sensor and a BarrettHand. An [example launch file](https://github.com/personalrobotics/libbarrett_ros/blob/ros_control/launch/wam_control.launch) for controlling HERB is available in the `launch/` directory. This loads ROS parameters from a [YAML file](https://github.com/personalrobotics/libbarrett_ros/blob/ros_control/config/wam_config.yaml) in the `config/` directory. HERB's libbarrett configuration is [available in the `herb_launch` repository](https://github.com/personalrobotics/herb_launch/tree/master/libbarrett_config).

You can start HERB's controller using `roslaunch`:
```bash
roslaunch libbarrett_ros wam_control.launch
```
