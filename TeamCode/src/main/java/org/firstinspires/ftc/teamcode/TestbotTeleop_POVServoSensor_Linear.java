/**
 * Acknowledgements
 * This work is based on version 7.1  of the FTCRRobotController programs developed by FIRST Tech Challenge.
 * As of July 22, 2022, these programs are no longer available and have been replaced by later versions.
 * Accordingly, I have referenced the latest, 7.2 version of each program.
 *      The actual program is based on (RobotTeleopPOV_Linear.java):
 *          https://tinyurl.com/2p8ehzt2
 *      Programming digital touch sensor (SensorDigitalTouch.java):
 *          https://tinyurl.com/2s47w2ph
 *      Programming a servo (ConceptScanServo.java):
 *          https://tinyurl.com/4dv5v6jc
 *      Programming color sensor (SensorColor.java) complicated???:
 *          https: https://tinyurl.com/yckzx5t9
 *      Programming Rev Robotics 2m distance sensor (SensorREV2mDistance.java):
 *          https://tinyurl.com/ytnbbn2r
 *
 * Additional resources that may be of use:
 *      REV Robotics touch sensor
 *          Information
 *              https://docs.revrobotics.com/touch-sensor
 *          OnBot Java programming
 *              https://docs.revrobotics.com/touch-sensor/application-examples
 *      REV Robotics servo
 *          Information
 *              https://docs.revrobotics.com/duo-build/actuators/servos
 *          SRS Programmer (for changing a Rev smart servo from fixed to continuous operation):
 *              https://docs.revrobotics.com/duo-build/actuators/servos/srs-programmer
 *          Video on how to program a servo:
 *              https://www.youtube.com/watch?v=e3KE4MELqus
 *      REV Robotics color sensor (V3)
 *          Information
 *              https://docs.revrobotics.com/color-sensor/
 *          OnBot Java programming
 *              https://docs.revrobotics.com/color-sensor/application-examples
 *      REV Robotics 2m Distance Sensor
 *          Information
 *              https://docs.revrobotics.com/2m-distance-sensor/
 */

package org.firstinspires.ftc.teamcode;/* Copyright (c) 2017 FIRST. All rights reserved.
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

//package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list

FTC Game Controller Reference
  https://docs.google.com/document/d/12gQTTJqDPs2JtyxIzylWSq6nD5qabHyXXto0CeTLYAA/edit

"x" Returns true if X button is pressed, false otherwise.
"y" Returns true if Y button is pressed, false otherwise.
"b" Returns true if B button is pressed, false otherwise.
"a" Returns true if A button is pressed, false otherwise.

"dpad_down" Returns true if D-pad Down is pressed, false otherwise.
"dpad_left" Returns true if D-pad Left is pressed, false otherwise.
"dpad_right" Returns true if D-pad Right is pressed, false otherwise.
"dpad_up" Returns true if D-pad Up is pressed, false otherwise

"left_trigger" Returns range of value 0.0 to 1.0 as right trigger is pressed.
"right_trigger" Returns range of value 0.0 to 1.0 as right trigger is pressed.
"left_bumper" Returns true if the left bumper is pressed, false otherwise.
"right_bumper" Returns true if the right bumper is pressed, false otherwise.

"back" Returns true if Back button is pressed, false otherwise.
"start" Returns true if the Start button is pressed, false otherwise.
"guide" or "mode" Returns true if the Guide/Mode button is pressed, false otherwise.

"left_stick_x" Returns the left-right deflection of the left stick.
  Negative values represent left deflections & positive right deflections. Range is -1.0 to +1.0.

"left_stick_y" Returns the up-down deflection of the left stick.
  Negative values represent up deflections & positive values down deflections. Range is -1.0 to +1.0.

"right_stick_x" Returns the left-right deflection of the right stick.
  Negative values represent left deflections & positive right deflections. Range is -1.0 to +1.0.

"right_stick_y" Returns the up-down deflection of the right stick,
  Negative values represent up deflections & positive values down deflections. Range is -1.0 to +1.0.

"left_stick_button" Returns true if the left stick button is pressed, false otherwise.

"right_stick_button" Returns true if the right stick button is pressed, false otherwise.

"atRest" Returns true if joys sticks and triggers in neutral position, false otherwise.
  atRest is a property you can access in your programming - not actually a “button” on the controller
*/

@TeleOp(name="Testbot: POV Servo Sensor", group="Testbot")
//@Disabled

    public class TestbotTeleop_POVServoSensor_Linear extends LinearOpMode {
        /* Declare OpMode members. */
        Testbot_Hardware robot          = new Testbot_Hardware();   // Use a Pushbot's hardware

    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;
        double max;
        double total_crservo;
        double servo_position = .5;
        final int no_sensor_action = 0;
        final int distance = 1;
        final int color = 2;
        final int distance_and_color = 3;
        int touch_action = no_sensor_action;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        //robot.colorV3.enableLed(false);  //Should turn off LED on color sensor, but does not work

        // Send telemetry message to indicate robot is ready to go:
        telemetry.addData("Press Play", "To continue");
        telemetry.update();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.

            drive = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;

            // Combine drive and turn for blended motion.
            left = drive + turn;
            right = drive - turn;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0) {
                left /= max;
                right /= max;
            }

            // Output the safe vales to the motor drives.
            robot.left_drive.setPower(left);
            robot.right_drive.setPower(right);


            //Use gamepad left & right Triggers to control rotation (CW or CCW) of cr_servo
            total_crservo = -gamepad1.left_trigger + gamepad1.right_trigger;
            robot.cr_servo.setPower(-gamepad1.left_trigger + gamepad1.right_trigger);


            //Use gamepad left & right Bumpers to control incremental rotation of servo
            //  DpadLeft  button set servo to +90 degrees (from vertical)
            //  DpadRight button set servo to -90 degrees (from vertical)
            //  Back button set servo to 0 degrees (from vertical)
            if (gamepad1.dpad_up | gamepad1.dpad_left | gamepad1.dpad_right) {
                if (gamepad1.dpad_up) {
                    telemetry.addData("dpad_up", "Pressed");
                    telemetry.update();
                    servo_position = 0.5;
                } else if (gamepad1.dpad_right)
                    servo_position = 0.0;
                else
                    // gamepad1.dpad_right is true
                    servo_position = 1.0;
                // Delay until all Dpad buttons released
                while (gamepad1.dpad_up | gamepad1.dpad_left | gamepad1.dpad_right);
                robot.servo.setPosition(servo_position);
            }

            //Use gamepad left & right Bumpers to control fixed positioning of servo
            //  Left Bumper decrease servo position by 9 degrees
            //  Right Bumper increase servo position by 9 degrees
            if (gamepad1.left_bumper | gamepad1.right_bumper) {
                if (gamepad1.right_bumper) {
                    servo_position -= 0.05;
                    if (servo_position < 0.0) {
                        servo_position = 0.0;
                    }
                    // Delay until right bumper button is released
                    while (gamepad1.right_bumper);
                } else {
                    // gamepad1.left_bumper is true
                    servo_position += 0.05;
                    if (servo_position > 1.0) {
                        servo_position = 1.0;
                    }
                    // Delay until left bumper button is released
                    while (gamepad1.left_bumper);
                }
                robot.servo.setPosition(servo_position);
            }


            // Send telemetry message to signify robot running;
            if (total_crservo != 0.0)
                telemetry.addData("crservo", "%.2f", total_crservo);
            else if (left != 0.0 || right != 0.0) {
                telemetry.addData("left wheel", "%.2f", left);
                telemetry.addData("right wheel", "%.2f", right);
            } else {
                // Display servo position
                // Note you display text comment by using  "telemetry.addData("", "%s", " ");"
                telemetry.addData("servo", "%.2f", servo_position);
            }

            if (robot.touch_sensor.isPressed()) {
                while (robot.touch_sensor.isPressed());
                if (touch_action != distance_and_color) {
                    touch_action++;
                }
                else {
                    touch_action = no_sensor_action;
                }
            }

            switch (touch_action) {
                case distance: {
                    //robot.color_sensor.enableLed(false);  //Should turn off LED on color sensor, but does not work
                    telemetry.addData("TOF Distance Sensor", " Distance (cm): %5.2f", robot.distance_sensor_right.getDistance(DistanceUnit.CM));
                    break;
                }
                case color: {
                    //robot.color_sensor.enableLed(true);  //Should turn on LED on color sensor, but does not work
                    telemetry.addData("Color Sensor", " Red: %3d, Green: %3d, Blue: %3d", robot.color_sensor.red(), robot.color_sensor.green(), robot.color_sensor.blue());
                    telemetry.addData("Color Sensor", " Distance (cm): %5.2f", robot.color_sensor.getDistance(DistanceUnit.CM));
                    break;
                }
                case distance_and_color: {
                    telemetry.addData("TOF Distance Sensor", " Distance (cm): %5.2f", robot.distance_sensor_right.getDistance(DistanceUnit.CM));
                    //robot.color_sensor.enableLed(true);  //Should turn on LED on color sensor, but does not work
                    telemetry.addData("Color Sensor", " Red: %3d, Green: %3d, Blue: %3d", robot.color_sensor.red(), robot.color_sensor.green(), robot.color_sensor.blue());
                    telemetry.addData("Color Sensor", " Distance (cm): %5.2f", robot.color_sensor.getDistance(DistanceUnit.CM));
                    break;
                }
                default: {
                    //No Sensor is selected, do nothing
                    break;
                }
            }
            telemetry.update();
            sleep(50);

        }
        //robot.color_sensor.enableLed(false);  //Should turn off LED on color sensor, but does not work
    }

}
