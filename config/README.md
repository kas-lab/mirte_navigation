# Config files

## Smac lattice configuration file

For more information about how to configure the lattice planner check: https://github.com/ros-navigation/navigation2/tree/main/nav2_smac_planner/lattice_primitives

### Generating lattice primitives for Mirte

First, clone `nav2` into a ROS workspace. It doesn't need to be in the same workspace as the `mirte_navigation` package. In fact, if you don't already need to build `nav2` from source, I would recommend using a separate workspace for this.

Create workspace and clone `nav2`:
```Bash
mkdir -p ~/ros_workspaces/nav2_ws/src 
git clone git@github.com:ros-navigation/navigation2.git -b humble
cd ~/ros_workspaces/nav2_ws/src
```
**Note:** Clone the appropriate version of the `nav2` repository, in the time of writing this README, we are using ROS 2 humble.

Install dependencies and build `nav2_smac_planner`:
```Bash
source ~/opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to nav2_smac_planner
```

Source the workspace:
```Bash
source install/setup.bash
cd src/navigation2/nav2_smac_planner/lattice_primitives/
``` 

Install the lattice generator dependencies:
```Bash
pip install -r requirements.txt
```

Run lattice generator:
```Bash
python3 generate_motion_primitives.py --config $HOME/ros_workspaces/rosa_ws/src/mirte_navigation/config/lattice_config.json --output $HOME/ros_workspaces/rosa_ws/src/mirte_navigation/params/lattice_output.json
```

**Note 2:** I'm not sure whether you can run the `generate_motion_primitives.py` without actually clonning the `nav2` repository, if you find out there is a way to do it, please update this README.