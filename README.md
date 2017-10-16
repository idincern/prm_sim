# prm_sim
ROS Probabilistic Road Map Simulation

## Instructions

After starting `roscore` the following nodes must be started:

- World map: `rosrun stage_ros stageros $(rospack find a3_help)/worlds/uoa_robotics_lab.world`
- Convert World map to OgMap: `rosrun local_map local_map /local_map/scan:=/base_scan_1 _map_width:=200 _map_height:=200 _map_resolution:=0.1`
- Convert OgMap to OpenCV image: `rosrun a3_help map_to_image_node map:=/local_map/local_map /pose:=/odom`
- Visualisation: `rosrun rviz rviz -d $(rospack find a3_help)/rviz/pfms.rviz`
- Viewing the image produced by the prm: `rqt_image_view` looking at topic `/map_image/full`.

## Building

- When first building run `catkin_make -j1` to auto-generate the src from the srv file.

TODO: The `a3_help` will change when I merge with this project.
