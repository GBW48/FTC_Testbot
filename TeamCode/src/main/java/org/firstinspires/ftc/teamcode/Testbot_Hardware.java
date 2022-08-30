/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * This class uses the following hardware configuration: 20201 Pushbot Program
 *
 * Control Hub
 *   Motors: REV Robotics Core Hex Motor    Port: 0  Motor name:  "right_drive"
 *   Motors: REV Robotics Core Hex Motor    Port: 1  Motor name:  "left_drive"
 *   Motors: REV Robotics Core Hex Motor    Port: 2  Motor name:  "arm"
 *   Motors: REV Robotics Core Hex Motor    Port: 3  Motor name:  "intake"
 *   Servos: Servo (standard)               Port: 0  Servo name:  "servo"
 *   Servos: Servo (continuous)             Port: 1  Servo name:  "cr_servo"
 *   i2C Bus 0: REV 2M Distance Sensor      Port: 0  Device name: "distance_sensor_left"
 *   i2C Bus 0: REV Expansion Hub IMU       Port: 2  Device name: "imu"
 *   i2C Bus 1: REV 2M Distance Sensor      Port: 0  Device name: "distance_sensor_right"
 *   i2C Bus 3: REV ColorV3 Sensor          Port: 0  Device name: "color_sensor"
 *   Digital Devices: REV Touch Sensor      Port: 1  Device name: "touch_sensor"
 * Expansion Hub 1
 *   Motors: REV Robotics 40:1 HD Hex Motor Port: 0  Motor name:  "duck"
 */
public class Testbot_Hardware
{
    /* Public OpMode members. */
    public DcMotor  right_drive                     = null;
    public DcMotor  left_drive                      = null;
    public Servo    servo                           = null;
    public CRServo  cr_servo                        = null;
    public DistanceSensor distance_sensor_left      = null;
    public DistanceSensor distance_sensor_right     = null;
    public Gyroscope imu                            = null;
    public TouchSensor touch_sensor                 = null;
    public ColorRangeSensor color_sensor            = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Testbot_Hardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define variables
        final double MIDDLE_POSITION = 0.5;
        final double NO_ROTATION = 0.0;

        // Define and Initialize Motors
        right_drive = hwMap.get(DcMotor.class, "right_drive");
        left_drive = hwMap.get(DcMotor.class, "left_drive");
        servo = hwMap.get(Servo.class, "servo");
        cr_servo = hwMap.get(CRServo.class, "cr_servo");
        distance_sensor_left = hwMap.get(DistanceSensor.class, "distance_sensor_left");
        distance_sensor_right = hwMap.get(DistanceSensor.class, "distance_sensor_right");
        imu = hwMap.get(Gyroscope.class, "imu");
        touch_sensor = hwMap.get(TouchSensor.class, "touch_sensor");
        color_sensor = hwMap.get(ColorRangeSensor.class, "color_sensor");

        right_drive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors

        // Set all motors to zero power
        left_drive.setPower(0.0);
        right_drive.setPower(0.0);


        // Set all motors to run with or without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        servo  = hwMap.get(Servo.class, "servo");
        cr_servo = hwMap.get(CRServo.class, "cr_servo");
        servo.setPosition(MIDDLE_POSITION);
        cr_servo.setPower(NO_ROTATION);
    }
 }

