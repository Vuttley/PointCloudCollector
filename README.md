# PointCloudCollector

Project from CERISE UFG

This project involves the development of a point cloud mapping device designed to support various research projects at CERISE. The system consists of a 2D lidar, a global shutter camera (not yet in use, but installed), an IMU (Inertial Measurement Unit), a Raspberry Pi 5, and two servos.

Sensors in Use:

STL27L (2D Lidar) <br>
WITMotion WT901 (IMU) <br>
VD66GY (for global shutter image capture) <br>

![scanner](https://github.com/user-attachments/assets/e2dfb998-2d62-4a44-aac4-890af0afb2f3)

![31f946f8-03cd-407b-bd87-9b17113b5fff](https://github.com/user-attachments/assets/31e84157-7a0b-4b43-9e1b-005a5ea453a6)

Our scanner is based on a pan-tilt approach, and the camera can be used later for colorization. <br>
Below is our first scan test, in a room at UFG.

<img width="2137" height="1179" alt="resultado" src="https://github.com/user-attachments/assets/3a9d4b16-0dca-4880-b64c-d54cb77a11c2" />

Our pipeline includes smoothing the IMU data with UnivariateSpline, outlier removal by filtering high angular velocity variations, and point downsampling.
