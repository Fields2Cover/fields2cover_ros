
> [!IMPORTANT]  
> As Nav2 released an actual proper wrapper for ROS (https://github.com/open-navigation/opennav_coverage), we have decided to unmantain this repository.



# Fields2Cover ROS
[![DOI](https://zenodo.org/badge/DOI/10.1109/LRA.2023.3248439.svg)](https://doi.org/10.1109/LRA.2023.3248439)


This package is an interface to ROS of the [Fields2Cover library](https://github.com/Fields2Cover/Fields2Cover)

<img src="logo_f2c.jpeg" width="250" height="250">

## Installation

This code has been tested with ROS 1 noetic and melodic, and ROS 2 galatic, humble and rolling.

### ROS 1
First, install [ROS noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) and create a [workspace (from now: catkin_ws/ )](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

From `catkin_ws/` clone and compile the package as:
```
git clone https://github.com/Fields2Cover/Fields2Cover src/fields2cover
git clone https://github.com/Fields2Cover/fields2cover_ros src/fields2cover_ros
rosdep install -r --ignore-src -y --from-paths .
catkin_make_isolated
```

### ROS 2

Install [ROS 2](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) and create your workspace as `mkdir ws`.

From `ws/` clone and compile the package as:
```
git clone https://github.com/Fields2Cover/Fields2Cover src/fields2cover
git clone https://github.com/Fields2Cover/fields2cover_ros src/fields2cover_ros
rosdep install -r --ignore-src -y --from-paths .
colcon build
```


## Running a demo (Only on ROS 1)

To run an interactive demo, run:

```
roslaunch fields2cover_ros view_field.launch
```

A rviz and a rqt_reconfigure windows will appear, so you can interactively modify the path created:

<img src="demo_image.png">


## Citing


Please cite the [following paper](https://doi.org/10.1109/LRA.2023.3248439) when using Fields2Cover for your research:

```
@article{Mier_Fields2Cover_An_open-source_2023,
  author = {Mier, Gonzalo and Valente, João and de Bruin, Sytze},
  doi={10.1109/LRA.2023.3248439},
  journal={IEEE Robotics and Automation Letters},
  title={Fields2Cover: An open-source coverage path planning library for unmanned agricultural vehicles},
  volume={8},
  number={4},
  pages={2166-2172},
  year = {2023}
}
```



## Credits and more info

This code repository is part of the project Fields2Cover which is (partly) financed by the Dutch Research Council (NWO).


