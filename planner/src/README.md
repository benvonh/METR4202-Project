# METR4202 Color Detection Help
This is sample code that can be used for the METR4202 project demo.

There are two files provided, both running on Python 3.

To clone the repository enter the following into the terminal:

```
git clone https://github.com/uqmvale6/metr4202_color.git
```

![alt text](https://github.com/uqmvale6/metr4202_color/blob/main/color_picker.png)


## Color Picker
This tool can be used with the XIMEA camera to select the BGR (Blue/Green/Red for OpenCV) values for the four block colors.

To use it, run the following, in the folder after cloning the repository.

```
python3 color_picker.py
```

Then, you can click on each quadrant to change selection and click on the camera image to modify the color of the quadrant.

When finished, click save and it will save to a configuration file ```colors.config```.


## Color Detector
This tool can be used with the XIMEA camera to detect the color of any pixel, given the colors picked in the previous tool.

Please modify this code to suit your project, using ROS, and make sure to give proper references to the author, i.e., me :-)

To run the demo to see how this works, use the following command in the terminal

```
python3 color_detector.py
```

The color detected will appear printed in the terminal.
