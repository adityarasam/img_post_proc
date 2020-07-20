This node subscribes to images in ros, converts them to CV compatible images, then performs image processing like Histogram Equilization, Feature Detection etc and finally publishes the processed image

Compiling instructions
Clone the repository in the src/ folder of your catkin workspace and then run following command from catkin_ws

$ catkin_make --pkg img_post_proc



Running the node

$ rosrun img_post_proc image_converter_post_proc 
