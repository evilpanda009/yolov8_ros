# yolov8_ros

### ROS wrapper for [yolov8_tracking](https://github.com/mikel-brostrom/yolov8_tracking)

### Usage  
1. Clone this repo in your `catkin_ws/src`
2. `cd yolov8_ros`
3. Make a copy of the `track.py` file in the folder `yolov8_tracking` and save it wherever you like.
4. Now delete the `yolov8_tracking` folder and clone a fresh copy from [yolov8_tracking](https://github.com/mikel-brostrom/yolov8_tracking) by following the instructions there. This step downloads the yolov8 repo as sub module
5. Now replace the new track.py file with the one you copied
6. `catkin build` or catkin_make in the folder `catkin_ws/src`
7. Now launch the `my_tracker.py` node (in the src folder) and the track.py script by specifying whatever source, classes etc. (for this refer [yolov8_tracking](https://github.com/mikel-brostrom/yolov8_tracking) )
