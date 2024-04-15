# ROS

## Useful Commands

- Check Ubuntu version `lsb_release -a`
- Check ROS version `rosversion -d`
- Open topic monitor `rqt`

To create an alias for a repetitive command, add the following line to the .bashrc file:

```bash
alias sb='source ~/.bashrc'
```

## Setup

### Create a ROS Workspace

Do the following steps to create a ROS workspace:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_make
```

Source the workspace, it's better to add this line to the .bashrc file:

```bash
source ~/catkin_ws/devel/setup.bash
```

### Create a Package

Move to the workspace source directory:

```bash
cd ~/catkin_ws/src
```

Create a package:

```bash
catkin_create_pkg {package_name} {dependencies}
# Most common dependencies: roscpp, rospy, std_msgs
```

Build the package:

```bash
cd ~/catkin_ws
catkin_make
```