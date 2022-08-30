/**
 * Acknowledgements
 * This work is based on version 7.1  of the FTCRRobotController programs developed by FIRST Tech Challenge.
 * As of July 22, 2022, these programs are no longer available and have been replaced by later versions.
 * Accordingly, I have referenced the latest, 7.2 version of each program. However, this program is no longer
 * available in the latest release. Accordingly, I have referenced the PushbotHardware.java program from
 * an earlier release.
 *      The actual program is based on (PushbotHardware.java):
 *          https://tinyurl.com/2s7vram5
 *
 * Additional resources that may be of use:
 *      REV Robotics Motors (Actuators)
 *          Motor Basics
 *              https://docs.revrobotics.com/duo-build/actuators/servos/srs-programmer
 *          Core Hex Motor:
 *              https://docs.revrobotics.com/duo-build/actuators/motors/core-hex-motor
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
// *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Testbot: Drive By Time", group="Testbot")
//@Disabled

public class TestbotAuto_DriveByTime_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    Testbot_Hardware robot          = new Testbot_Hardware();   // Enable to use Testbot's hardware definition

    private ElapsedTime     runtime = new ElapsedTime();

    static double     FORWARD_SPEED = 0.5;
    static double     TURN_SPEED    = 0.4;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to indicate robot is ready to go:
        telemetry.addData("Press Play", "To continue");
        telemetry.update();

        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        //In Autonomous mode, nun continuously, until 30 second timer expires

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

        robot.left_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);   // Set the left motor to not use the encoder
        robot.right_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // Set the right motor to not use the encoder

        while (opModeIsActive()){
            timeDrive(FORWARD_SPEED,FORWARD_SPEED,3.0,"Drive forward");         // Step 1: Drive forward for 3 seconds
            timeDrive(TURN_SPEED,-TURN_SPEED,2.2,"Turn CW");                    // Step 2: Turn right (CW) for 2.2 seconds  // Different times, because
            timeDrive(-TURN_SPEED,TURN_SPEED,2.4,"Turn CCW");                   // Step 3: Turn left (CCW) for 2.4 seconds  // motors are not identical
            timeDrive(-FORWARD_SPEED,-FORWARD_SPEED,3.0,"Drive backward");      // Step 4: Drive backward for 3 seconds

            telemetry.addData("Status", "Robot has driven back to the starting location");
            telemetry.addData("Status", "Drive operation, using encoders is complete");
            telemetry.update();
            sleep(4000);
        }

    }
    /**
     * The timeDrive Method uses setPower to drive the left and right wheels of the robot
     *
     * @param   left_speed   Desired speed of left motor
     * @param   right_speed  Desired speed of right motor
     * @param   time_to_run  Amount of time motors will run
     * @param   description  Text for telemetry description
     * @return  nothing is returned
     */
    public void timeDrive(double left_speed,
                          double right_speed,
                          double time_to_run,
                          String description){
        robot.left_drive.setPower(left_speed);
        robot.right_drive.setPower(right_speed);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time_to_run)) {
            telemetry.addData("Status", "%s with %1.1f of %1.1f seconds remaining", description, runtime.seconds(),time_to_run);
            telemetry.update();
        }
        robot.left_drive.setPower(0);
        robot.right_drive.setPower(0);
        sleep(4000);
    }
}
