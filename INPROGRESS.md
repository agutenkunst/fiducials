# INPROGRESS

Currently Porting to foxy


## Deps


## Usage
### Create markers
Still a bit fuzzy with the correct deps
```
ros2 run aruco_detect create_markers.py 101 102 foo.pdf
```

### Perform detection

With https://github.com/agutenkunst/ros2_usb_camera/tree/fiducials_migration

(check that corrected calibration file is attached)
```
ros2 launch usb_camera_driver usb_camera_node.launch.py
```
Note: Just using some "valid" calibration data


```
ros2 launch usb_camera_driver usb_camera_node.launch.py
```

Check images in rviz with
```
ros2 run rviz2 rviz2 -d src/fiducials/rviz_fiducials_config.rviz
```

## Pitfalls
- For some reason the default subscription QoS of is default not sensor. Due to this one has to make sure the image is send with a compatible profile. See https://github.com/ros-perception/image_common/issues/156

## Done
- [x] Migrate launch file

## Doing
- [ ] Porting dynamic_reconfigure
- [ ] Check results in `/fiducial_images`

## TODO
- [ ] Proper topic remapping in launch file
- [ ] Check that param setting behaves as in ROS1
- [x] `create_markers.py`
- [ ] Port package fiducial_slam

## Open Questions
- What calibration data has to be used on the camera package if the calibration is yet to be performed? Did I do this wrong?

## Attachments
camera_calibration
```
image_width: 1280
image_height: 720
camera_name: camera
camera_matrix:
  rows: 3
  cols: 3
  data: [3448.252514, 0.000000, 632.017271, 0.000000, 3451.884113, 93.305465, 0.000000, 0.000000, 1.000000]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [1.479832, -7.527908, -0.040732, -0.131598, 0.000000]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 0.000000, 1.000000]
projection_matrix:
  rows: 3
  cols: 4
  data: [3597.542236, 0.000000, 582.551711, 0.000000, 0.000000, 3726.241211, 89.189268, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000]
```