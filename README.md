
# Camera Calibration

Camera calibration is the process of determining specific camera parameters in order to complete operations with specified performance measurements. Camera calibration can be defined as the technique of estimating the characteristics of a camera. It means that we have all of the camera’s information like parameters or coefficients which are needed to determine an accurate relationship between a 3D point in the real world and its corresponding 2D projection in the image acquired by that calibrated camera.

## Setup

* We calibrated a mobile rear camera (make type ASUS ZenFone max pro M1) using checkerboard pattern. We measured physically world coordinates of the checkerboard by scale setting a world coordinate system and the corresponding image coordinates were measured from captured image.
* We have used two checkerboard printouts. Each checkerboard printout has 8*8 black & white squares. Size of each square on these printouts is 2cm x 2 cm.
* The checkerboard printouts were pasted on two walls meeting at 90 degrees to each other as shown in fig-1. The world coordinate system setup is also shown wherein Z axis is pointing up, X axis is pointing to right and Y axis is pointing to left.

## Visualization

* Checkboard

![GitHub Logo](/images/Capture1.PNG)

* 2D points marked by us (red points) and the estimated 2D projections of the marked 3D points (yellow points)

![GitHub Logo](/images/Capture2.PNG)

* Estimated 2D projections of all the checkerboard corners

![GitHub Logo](/images/Capture3.PNG)
