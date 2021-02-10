# INPROGRESS

Currently Porting to foxy

## Usage
With https://github.com/agutenkunst/ros2_usb_camera/tree/fiducials_migration
```
ros2 run usb_camera_driver usb_camera_driver_node __ns:=/camera_ns __params:=/home/alex/ros2_ws_fiducials/src/ros2_usb_camera/config/config.yaml
```
Note: Just using some "valid" calibration data


```
ros2 launch aruco_detect aruco_detect.launch.py
```

Check images in rviz with
## Done
- [x] Migrate launch file

## Doing
- [ ] Porting dynamic_reconfigure
- [ ] Check results in `/fiducial_images`

## TODO
- [ ] Check that param setting behaves as in ROS1
- [ ] `create_markers.py`
- [ ] Port package fiducial_slam