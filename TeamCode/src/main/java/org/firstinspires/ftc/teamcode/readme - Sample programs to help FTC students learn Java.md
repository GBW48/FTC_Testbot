## A series of Java programs to help FIRST Tech Challenge (FTC) students learn Java

This code repository aims to help FIRST Tech Challenge (FTC) students learn Java.
The programs are written and compiled using Android Studio. Successfully compiled programs are uploaded to a student-designed robot called a Testbot to complete a series of tasks. The Testbot supports various devices, including two REV core hex motors with encoders; continuous and standard servos; REV touch, distance, and color sensors; an Inertial Measurement Unit (IMU); and a Logitech 920c webcam.


Most of the programs are based on sample programs provided by FTC.
https://tinyurl.com/k64twk9v

The TensorFlow program is based on the work by Dr. Ed Epp to demonstrate the use of
TensorFlow for object recognition in FTC challenges.
https://github.com/edcepp/FTCEppTensorCode/tree/master/FTCEppTensorCode

The OpenCV programs are based on the SkystoneCVTutorial developed by wolfcorpftc.
https://github.com/wolfcorpftc/SkystoneCVTutorial
In addition, these programs use EasyOpenCV, developed by OpenFTC.  EasyOpenCV provides an easy way to use OpenCV on an FTC robot.
https://github.com/OpenFTC/EasyOpenCV#installation-instructions-android-studio

All of the programs use linear as opposed to the standard, iterative style of OpMode
programming; because, in the author’s opinion, it is easier for novice programmers to understand. The Teleop programs demonstrate how to control and interact with the robot’s motors, encoders, servos, sensors, IMU, and webcam. Several of the programs are based on the sample code provided by FTC. While other programs use material from other sources, including REV Robotics and various websites. The sample code has been modified to support the Testbot hardware class, use different hardware, provide additional features or improve readability.  


The following programs were tested using our team’s robot, which was created to compete in the 2021 – 2022 FTC season.

### Testbot_Hardware.java
- Not an OpMode, but a class used to define a robot's hardware. In our case, that robot is the Testbot. The other programs use this hardware class.

### TestbotAuto_DriveByTime_Linear.java
- Using time; demonstrates forward, backward, and turning movements

### TestbotAuto_DriveByEncoder_Linear.java
- Using motor encoders; demonstrates forward, backward, and turning movements

### TestbotTeleop_POVServoSensor_Linear.java
- Illustrates using a gamepad to drive the robot using point-of-view or arcade-style
steering. In addition, the gamepad controls the rotation of standard and ​continuous servos.

- Uses a touch sensor to control the selection and use of color and distance sensors. Both color and distance information are displayed on the driver’s station.

### TestbotTeleop_DriveByGyro_Linear.java
- Using motor encoders and the IMU (Inertial Measurement Unit); demonstrates forward, backward, and turning movements, and driving in a square

### TestbotTeleop_ObjectDetectionTensorFlowLite_Linear.java
- Using TensorFlow Lite and an FTC-provided object detection model, the program demonstrates the detection of white ball(s), yellow cube(s), or yellow duck(s) when placed on a dark surface. Placing objects on a dark surface improves overall detection rates.

- Provides information about each detected object’s size and location on the android screen

### OpenCVAuto_FindSkystones_Linear
- The program uses information from a  webcam to analyze two adjacent 2-inch yellow cubes called "stones." A cube with a black face is called a "skystone."

- A specialized image analysis program (SkystoneDetector.java) uses OpenCV to define two regions of interest (ROIs) in the webcam viewing frame. Information from each ROI is used to determine the presence of a single skystone.

- With information from the detector, the program can identify the position of a single skystone in the viewing area. The following skystone detection information is returned: LEFT, RIGHT, or NOT_FOUND.

### OpenCVAuto_FindSkystonesEnhanced_Linear

- This is an enhanced version of the OpenCVAuto_FindSkystones_Linear.java program. The program can now detect two skystones.

- This is made possible through an enhanced image analysis program (SkystoneDetectorEnhanced.java). The detector uses OpenCV to define four regions of interest in the webcam viewing frame. Information from each ROI is used to determine the presence of up to two skystones. The detector ignores the case where there are only yellow or no stones in the viewing area.

- With information from the enhanced detector, the program can now identify the position of any skystone in the viewing area. The following skystone detection information is returned: LEFT, RIGHT, LEFT_AND_RIGHT or NOT_FOUND.

### OpenCVTeleop_PositionROI_Linear.java

- The primary purpose of this program is to provide a “tool” that helps a user interactively define a Region Of Interest (ROI). This ROI defines “an area of interest” within a video image provided by an attached webcam. When viewed on the driver station, a green rectangle identifies an ROI.
