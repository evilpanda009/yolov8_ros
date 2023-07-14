# yolov8_ros

### ROS wrapper for [yolov8_tracking](https://github.com/mikel-brostrom/yolov8_tracking)

This is a ROS compatible version of the [yolov8_tracking](https://github.com/mikel-brostrom/yolov8_tracking) repository, which provides the option to use a host of tracking algorithms on the output of yolo models for robust and real-time detection, segmentation and tracking of upto 20 classes of objects. The information of the resulting bounding boxes is published over a ROS node to be used by the controller to navigate the robot using visual servoing. 
### Usage  
1. Clone this repo in your `catkin_ws/src`
2. `cd yolov8_ros`
3. Make a copy of the `track.py` file in the folder `yolov8_tracking` and save it wherever you like.
4. Now delete the `yolov8_tracking` folder and clone a fresh copy from [yolov8_tracking](https://github.com/mikel-brostrom/yolov8_tracking) by following the instructions there. This step downloads the yolov8 repo as submodule
5. Now replace the new track.py file with the one you copied
6. `catkin build` or catkin_make in the folder `catkin_ws/src`
7. Now launch the `my_tracker.py` node (in the src folder) and the track.py script by specifying whatever source, classes etc. (for this refer [yolov8_tracking](https://github.com/mikel-brostrom/yolov8_tracking) )

## Further  

Though the monocular framework (fused with 2D Lidar stream of a simple rplidar) gives pretty good results, this can be further improved. Depth data from an intel realsense or ZED stereo-camera can greatly improve performance and obstacle avoidance capabilities. Also currently, the controller latches onto the first person to enter the frame (after a specific time of no detections), however one can choose which person(or object) to follow by simply providing the ID number of the desired bounding box, or writing some script to use natural language commands to decide and pick an ID (eg. "Follow the dog or the guy in the red shirt"), or even use some pre trained identification model to always follow a specific "owner(s)".

https://github.com/evilpanda009/yolov8_ros/assets/70006069/22fa9bdd-adfd-439c-9154-d18389434dc5

