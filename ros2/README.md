# Task 6:

The following tasks assume that you have completed successfully the demo found at: https://github.com/tingelst/ros2_seminar_spring_2020_demos

## Implement a camera node using OpenCV

1. Create a ROS2 Python package named `tpk4128_opencv_camera` with a node named `opencv_camera_node`.

2. Implement the class `OpenCVCameraNode` which inherits form `rclpy.Node`. Using OpenCV, the node shall grab an image from a webcam with a frequency of 10 Hz and publish the image on the topic `\image` with message type `sensor_msgs.msg.Image`.

3. Verify that the published image is correct using `rqt_image_view` and selecting the topic `\image`. Open `rqt_image_view` by running
```bash
ros2 run rqt_image_view rqt_image_view 
```