## Executables:

### save_kinect_cloud_to_file

Optional parameters in the nodes private namespace:
 * `ROSHelpers::GetParam<std::string>(ph_, "point_cloud_topic", "/kinect2_victor_head/hd/points")`
 * `ROSHelpers::GetParam<std::string>(ph_, "target_frame", "victor_root")`
 * `ROSHelpers::GetParam<std::string>(ph_, "file_name_prefix", "./logs/point_cloud")`


Needed launch files to work by default:
 * roslaunch kinect2_calibration_files kinect2_bridge_loki.launch
 * roslaunch lightweight_vicon_bridge vicon_bridge.launch
 * roslaunch lightweight_vicon_bridge mocap_to_kinect_static_transforms.launch
 * roslaunch lightweight_vicon_bridge mocap_to_victor_static_transform.launch