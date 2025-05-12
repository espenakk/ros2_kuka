# ROS2 workspace for kuka-ballcatcher project. 

## Kuka control

*Commands ball trajectory prediction*
```bash
ros2 launch kuka_control ball_trajectory_prediction.launch.py test_mode:=true
```

## Kuka rsi io library

### Building with colcon and vcpkg

This package uses [vcpkg](https://github.com/microsoft/vcpkg) for dependency management.  

*To build with colcon, we need to specify the vcpkg toolchain*:
```
colcon build --packages-select kuka_rsi_io --symlink-install --cmake-args -DCMAKE_TOOLCHAIN_FILE=$HOME/vcpkg/scripts/buildsystems/vcpkg.cmake
```