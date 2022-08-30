/**
 * Acknowledgements
 * This work is based on version 7.1  of the FTCRRobotController programs developed by FIRST Tech Challenge.
 * As of July 22, 2022, these programs are no longer available and have been replaced by later versions.
 * Accordingly, I have referenced the latest, 7.2 version of each program.
 *      The actual program is based on (RobotAutoDriveByEncoder_Linear.java):
 *          https://tinyurl.com/5c9sbdx3
 *
 * Additional resources that may be of use:
 *      Videos
 *          Encoder tutorials
 *              https://www.youtube.com/watch?v=d0liBxZCtrA
 *          Driving precised distances with encoders
 *              https://www.youtube.com/watch?v=Y0OZCdOLhwo
 *      REV Robotics Motors (Actuators)
 *          Motor Basics
 *              https://docs.revrobotics.com/duo-build/actuators/servos/srs-programmer
 *          Core Hex Motor:
 *              https://docs.revrobotics.com/duo-build/actuators/motors/core-hex-motor
 *      REV Robotics Encoders
 *          Information
 *              https://docs.revrobotics.com/duo-control/sensors/encoders
 *          Motor Encoder Specifications
 *              https://docs.revrobotics.com/duo-control/sensors/encoders/motor-based-encoders
 */

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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 36i inches
 *   - Turn in place between left and right wheels (CW) for 18 inches
 *   - Drive Backwards for 24 inches
 *   - Turn around right wheel (CW) for 18 Inches
 *   - Turn around left wheel (CCW) for 18 Inches
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Testbot: Drive By Encoder", group="Testbot")
//@Disabled

public class TestbotAuto_DriveByEncoder_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    Testbot_Hardware robot          = new Testbot_Hardware();   // Use Testbot's hardware definition


    private ElapsedTime     runtime = new ElapsedTime();

    /*
    REV Robotics Core Hex Motor Specifications:
        Gear Ratio: 72:1
        Encoder Counts per Revolution
            At the motor - 4 counts/revolution
            At the output - 288 counts/revolution

    REV Robotics 90MM Grip Wheel Specifications:
        Diameter: 90mm/3.54in
    */

    static final double     COUNTS_PER_MOTOR_REV    = 4 ;       // REV Robotics Core Hex Motor
    static final double     DRIVE_GEAR_REDUCTION    = 72 ;      // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.54 ;    // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.5;
    static final double     TURN_SPEED              = 0.4;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        /**
         Notes on using DC Motor Controller arguments (https://tinyurl.com/48k6ht9h)
         Reset Encoders:
         The robot will NOT move in this mode. It's used to reset the encoder values back to zero.

         Run without Encoders:
         This is open-loop speed control. This means that setPower() simply sets the voltage level applied to the motors.
         How fast you move depends on battery voltage, robot friction, slope etc.

         Run with Encoders:
         This is closed-loop speed control. Encoders are required for this mode. SetPower() is actually requesting a certain speed,
         based on the top speed of encoder 4000 pulses per second.
         The Motor controllers automatically adjust the voltage to the motors to obtain the requested speed. This is like a car's cruise control.
         It's great at producing slow speeds. It's also great for making both sides of the robot run at the same speed to get straighter driving.

         Run To Position.
         This is closed-loop position control. Encoders are required for this mode. setPower() is simply saying what the top speed is to reach the
         desired encoder position. To control the movement of the robot you actually set the target encoder position for each motor.
         The motor controller will provide power to the motors in order to get them to reach the requested position
         (may be a short or very long distance) is quickly as possible, and then hold that position..

         Each motor can only be in one of these modes.

         For additional information on DC Motors reference the following information about the DcMotor.RunMode Enum
         https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/DcMotor.RunMode.html
         */

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Status",  "Starting encoder values, Left wheel: %5d right wheel :%5d",
                          robot.left_drive.getCurrentPosition(),
                          robot.right_drive.getCurrentPosition());
        // Send telemetry message to indicate robot is ready to go:
        telemetry.addData("Press Play", "To continue");
        telemetry.update();

        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED,  15,  15, 4.0, "Drive forward for 15\"");                 // Step 1: Forward 18 Inches with 5 Sec timeout
        encoderDrive(TURN_SPEED,   13, -13, 4.0, "Turn in place CW for 12\"");              // Step 2: Turn in place between left and right wheels (CW) for 12 Inches with 4 Sec timeout
        encoderDrive(TURN_SPEED,   -13, 13, 4.0, "Turn in place CCW for 12\"");             // Step 3: Turn in place between left and right wheels (CCW) for 12 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, -17, -17, 4.0, "Drive in reverse for 17\"");              // Step 4: Reverse 18 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED,  13,   0, 4.0, "Turn around right wheel (CW) for 13\"");  // Step 5: Turn around right wheel (CW) for 18 Inches with 4 Sec timeout
        encoderDrive(TURN_SPEED,   -9, -9, 4.0, "Drive in reverse for 9\"");                // Step 6: Reverse 9 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED,   0,  13, 4.0, "Turn around left wheel (CCW) for 13\"");  // Step 7: Turn around left wheel (CCW) for 18 Inches with 4 Sec timeout
        encoderDrive(TURN_SPEED,   -5.75, -5.75, 4.0, "Drive in reverse for 5.75\"");       // Step 8: Reverse 9 Inches with 4 Sec timeout

        telemetry.addData("Status", "Robot has driven back to the starting location");
        telemetry.addData("Status", "Drive operation, using encoders is complete");
        telemetry.update();
        sleep(5000);
    }

    /**
     *  The encoderDrive Method performs a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     *
     * @param   speed       Speed that will be used for both motors
     * @param   leftInches  Inches that left wheel will travel
     * @param   rightInches Inches that left wheel will travel
     * @param   timeout     Timeout for drive operation
     * @param   description Text for telemetry description
     * @return  nothing is returned
     */
    public void encoderDrive(double speed,
                             double leftInches,
                             double rightInches,
                             double timeout,
                             String description) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.left_drive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.right_drive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.left_drive.setTargetPosition(newLeftTarget);
            robot.right_drive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.left_drive.setPower(Math.abs(speed));
            robot.right_drive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            if (leftInches > 0 && rightInches > 0) {
                // Handles case when both of the motors are set to non-zero power
                while (opModeIsActive() &&
                        (runtime.seconds() < timeout) &&
                        (robot.left_drive.isBusy() &&  robot.right_drive.isBusy())) {

                    // Display information for the driver.
                    telemetry.addData("Action", description);
                    telemetry.addData("Target encoder values",  "Left side: %5d Right side: %5d", newLeftTarget,  newRightTarget);
                    telemetry.addData("Current encoder values","Left side: %5d Right side: %5d",
                            robot.left_drive.getCurrentPosition(),
                            robot.right_drive.getCurrentPosition());
                    telemetry.update();
                }
            }
            else {
                // Handles case when one of the motors is set to zero power
                while (opModeIsActive() &&
                        (runtime.seconds() < timeout) &&
                        (robot.left_drive.isBusy() || robot.right_drive.isBusy())) {

                    // Display information for the driver.
                    telemetry.addData("Action", description);
                    telemetry.addData("Target encoder values",  "Left side: %5d Right side: %5d", newLeftTarget,  newRightTarget);
                    telemetry.addData("Current encoder values","Left side: %5d Right side: %5d",
                            robot.left_drive.getCurrentPosition(),
                            robot.right_drive.getCurrentPosition());
                    telemetry.update();
                }
            }
            // encoderDrive operation is complete
            // Stop both motors and stop using RUN_TO_POSITION mode, by opening the motors to RUN_USING_ENCODER mode

            // Stop all motion;
            robot.left_drive.setPower(0);
            robot.right_drive.setPower(0);
            sleep(400);   // pause after stop

            // Turn off RUN_TO_POSITION
            robot.left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            sleep(200);   // optional pause after each move

            // Turn off RUN_TO_POSITION
            robot.left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(200);   // optional pause after each move

        }
    }

}
