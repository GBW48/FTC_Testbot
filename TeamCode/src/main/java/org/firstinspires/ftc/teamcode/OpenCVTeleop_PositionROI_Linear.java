/**
 * Acknowledgements
 * This program used some of the concepts from the SkystoneCVTutorial which was developed by wolfcorpftc
 * In particular, the use of instance variables and getters which allow: the OpMode class to access
 * information defined in the pipeline class, and the pipeline class to access information defined
 * in the OpMode class.
 *      The code can be found at:
 *          https://github.com/wolfcorpftc/SkystoneCVTutorial
 *      In addition a video tutorial can be found at:
 *          https://www.youtube.com/watch?v=JO7dqzJi8lw
 *
 *In addition, this program uses EasyOpenCV which was developed by OpenFTC
 *      Information about EasyOpenCV can be found at:
 *          https://github.com/OpenFTC/EasyOpenCV#installation-instructions-android-studio
 */

/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;


@TeleOp(name="OpenCV: Position Region Of Interest", group="Vision")
//@Disabled


// *************************************************************************************************
//
//                                      OpenCVTeleop_PositionROI_Linear class
//                       (For purposes of discussion, this is also called the "OpMode" class)
//
// *************************************************************************************************

public class OpenCVTeleop_PositionROI_Linear extends LinearOpMode
{

    OpenCvWebcam webcam;
    SamplePipeline pipeline;

    //IMPORTANT NOTE: Allows information defined in the OpMode class to be accessed from the Pipeline class
    //Declare instance variables that support the sharing of information between the two classes
    /*
     * rectangle_opmode[0] - Point A, X
     * rectangle_opmode[1] - Point A, Y
     * rectangle_opmode[3] - Point B, X
     * rectangle_opmode[4] - Point B, Y
     */
    int [] rectangle_opmode = {0, 0, 0, 0};
    int [] saved_rectangle_opmode = {0, 0, 0, 0};
    boolean valid_saved_rectangle_opmode_settings = false;

    static int webcam_width = 320;
    static int webcam_height = 240;
    static int width_increment = 10;
    static int height_increment = 5;
    public int i;
    public boolean first_time;
    public boolean button_summary;
    public boolean webcam_pipeline_running;
    public boolean valid_input;
    public boolean rectangle_txt_has_been_deleted;

    enum Action {
        WRITE_POINT_A,
        WRITE_POINT_B,
        WRITE_FILE,
        WRITE_FILE_THEN_EXIT,
        STOP_WEBCAM_PIPELINE,
        STOP_PROGRAM,
        DO_NOTHING
    }
    public Action current_operation = Action.WRITE_POINT_A;
    public Action previous_operation = Action.DO_NOTHING;

    public String line_of_text;
    public String file_name = "/FIRST/rectangle.txt";
    public File file_dir = Environment.getExternalStorageDirectory();
    public File file_path = new File(file_dir,file_name);


    @Override
    public void runOpMode()
    {
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using a webcam. Note that you will need to
         * make sure you have added the webcam to your configuration file and
         * adjusted the name here to match what you named it in said config file.
         *
         * We pass it the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        //Change to allow information from the pipeline class to be accessed from the OpMode class
        pipeline = new SamplePipeline();
        webcam.setPipeline(pipeline);

        /*
         * Open the connection to the camera device. New in v1.4.0 is the ability
         * to open the camera asynchronously, and this is now the recommended way
         * to do it. The benefits of opening async include faster init time, and
         * better behavior when pressing stop during init (i.e. less of a chance
         * of tripping the stuck watchdog)
         *
         * If you really want to open synchronously, the old method is still available.
         */
        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                webcam_pipeline_running = true;
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        // Send telemetry message to indicate robot is ready to go:
        telemetry.addData("On phone press '3 vertical dots' then 'Camera Stream'","");
        telemetry.addData("          ", "To display objects");
        telemetry.addData("Or press Play","");
        telemetry.addData("          ", "To identify objects");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            //
            // Initialize rectangle_opmode array that holds the rectangle's Point A and Point B values
            //
            file_dir = Environment.getExternalStorageDirectory();
            file_path = new File(file_dir, file_name);
            rectangle_txt_has_been_deleted = false;
            if (!file_path.exists()) {
                valid_input = false;
                while (true) {  // Outer loop used for confirmation of entry
                    // rectangle.txt does not exist
                    //  Based on user input create a rectangle.txt with a small or medium ROI
                    telemetry.addData("", "Rectangle.txt does not exist");
                    telemetry.addData("", "Create Rectangle.txt that defines a:");
                    telemetry.addData("Press b", " Small ROI (80x40)");
                    telemetry.addData("Press y", " Medium ROI (160x80)");
                    telemetry.update();
                    
                    while (true) {  // Inner loop used to process b or y button press
                        if (gamepad1.b) {
                            setup_ROI ("Small ROI settings", "New default rectangle.txt created", " with Small ROI settings",
                                    120, 100, 200, 140);
                            break;
                        } else if (gamepad1.y) {
                            setup_ROI ("Medium ROI settings", "Rectangle.txt created ", "  with Medium ROI settings",
                                    80, 80, 240, 160);
                            break;
                        }
                    }

                    // Delay until buttons b and y are released
                    while (gamepad1.b || gamepad1.y);
                    
                    // Check the user confirmed confirmed the b or y request
                    if (valid_input) {
                        break;  // Input validated, continue with processing

                    } else {
                                // Repeat the b or y request again
                    }
                }

            } else {
                // The file rectangle.txt exists.
                // Read rectangle.txt for the Point A and Point B position values of the rectangle.
                telemetry.addData("Rectangle.txt exists, with settings of", "");
                try {
                    FileReader reader = new FileReader(file_path);
                    BufferedReader bufferedReader = new BufferedReader(reader);
                    i = 0;
                    while ((line_of_text = bufferedReader.readLine()) != null) {
                        if (i <= 3) {
                            rectangle_opmode[i] = Integer.parseInt(line_of_text);
                        } else {
                            break;
                        }
                        i++;
                    }
                    bufferedReader.close();
                    reader.close();
                } catch (IOException e) {
                    telemetry.addData("File Read IOException", e);
                    telemetry.addData("Press back button to acknowledge", "");
                    telemetry.update();
                    while (!gamepad1.back) ;
                }

                valid_input = false;
                while (true) {  // Outer loop used for confirmation of entry
                    telemetry.addData("              ", " Ax: %3d, Ay: %3d   Bx: %3d, By: %3d",
                            rectangle_opmode[0], rectangle_opmode[1], rectangle_opmode[2], rectangle_opmode[3]);
                    telemetry.addData("ROI       ", " Width x Height: %3d x %3d",
                            Math.abs(rectangle_opmode[2] - rectangle_opmode[0]), Math.abs(rectangle_opmode[3] - rectangle_opmode[1]));
                    telemetry.addData("Press a", " To USE the Current ROI settings");
                    telemetry.addData("Press b", " To CREATE a Small ROI (80x40)");
                    telemetry.addData("Press y", "                           Medium ROI (160x80)");
                    telemetry.addData("Press x", " To DELETE rectangle.txt");

                    telemetry.update();
                    while (true) {  // Inner loop used to process a, b, y or x button press
                        if (gamepad1.a) {
                            if (is_input_valid("use of rectangle.txt")) {
                                valid_input = true;  // Response confirmed, continue processing
                                for (i = 0; i < saved_rectangle_opmode.length; i++) {
                                    // Copy the rectangle settings to the saved rectangle settings
                                    saved_rectangle_opmode[i] = rectangle_opmode[i];
                                }
                            } else {
                                valid_input = false;  // Ask the question again
                            }
                            break;
                        } else if (gamepad1.b) {
                            setup_ROI ("Small ROI settings", "New default rectangle.txt created", " with Small ROI settings",
                                120, 100, 200, 140);
                            break;
                        } else if (gamepad1.y) {
                            setup_ROI ("Medium ROI settings", "Rectangle.txt created ", "  with Medium ROI settings",
                                    80, 80, 240, 160);
                            break;
                        } else if (gamepad1.x) {
                            // Delete rectangle.txt
                            if (is_input_valid("rectangle.txt deletion")) {
                                valid_input = true;  // Response confirmed, continue processing
                            } else {
                                valid_input = false;  // Ask the question again
                                break;
                            }
                            if (file_path.exists()) {
                                try {
                                    file_path.delete();
                                    rectangle_txt_has_been_deleted = true;
                                } catch (Exception e) {
                                    telemetry.addData("File Deletion  IOException", e);
                                    telemetry.addData("Press back button to acknowledge", "");
                                    telemetry.update();
                                    while (!gamepad1.back) ;
                                }
                            }
                            break;
                        }
                    }

                    // Delay until buttons a, b, y and x are released
                    while (gamepad1.a || gamepad1.b || gamepad1.y || gamepad1.x) ;

                    // Check the user confirmed confirmed the a, b, y or x request
                    if (valid_input) {
                        break;  // Input validated, continue with processing

                    } else {
                                // Repeat the a, b, y or x request again
                    }
                }
            }
            // Initialize boolean control variables
            valid_saved_rectangle_opmode_settings = true;
            first_time = true;
            button_summary = true;
            current_operation = Action.WRITE_POINT_A;  // Reset current_operation to update Point A coordinates
            previous_operation = Action.DO_NOTHING;

            telemetry.addData("Press start button"," to continue");
            telemetry.update();
            while (!gamepad1.start);  // Delay until button a has been released
            while (gamepad1.start);
        }

        while (opModeIsActive()) {
            if (rectangle_txt_has_been_deleted) {
                // rectangle.txt has been deleted, exit the program
                // It is necessary to restart (power cycle) the robot, for the webcam to work
                telemetry.addData("Press start button to acknowledge","");
                telemetry.addData("", "Rectangle.txt has been deleted");
                telemetry.addData("", "You need to restart the robot");
                telemetry.update();
                while (!gamepad1.start);  // Delay until the operation has been acknowledged
                break;
            }
            if (button_summary) {
                telemetry.addData("Press start button to continue","");
                telemetry.addData("Button actions","");
                telemetry.addData("                         a"," To update Ax, Ay position of Rectangle");
                telemetry.addData("                         b"," To update Bx, By position of Rectangle");
                telemetry.addData("                         y"," To update rectangle.txt");
                telemetry.addData("                         x"," To update rectangle.txt then exit");
                telemetry.addData("right bumper   "," To stop webcam pipeline");
                telemetry.addData("logitech            "," To re-display Button actions");
                telemetry.update();
                button_summary = false;
                while (!gamepad1.start);
                while (gamepad1.start);
            }

            // Use gamepad1 a, b, y, x, logitech (ps), or mode (guide) to control the specification of:
            //  "a"           - Point A of the rectangle (X and Y values)
            //  "b"           - Point B of the rectangle (X and Y values)
            //  "y"           - Write out Point A and Point B values to rectangular.txt
            //  "x"           - Write out Point A and Point B values to rectangular.txt and Exit the program
            //  "guide"       - Stop running webcam pipeline
            //  "logitech"    - Re-display Button actions
            if (gamepad1.a || gamepad1.b || gamepad1.y || gamepad1.x || gamepad1.right_bumper || gamepad1.ps) {
                if (gamepad1.a) {
                    // Specify Point Ax & Ay values
                    if (current_operation == Action.WRITE_POINT_A && previous_operation == Action.DO_NOTHING ) {
                        first_time = true;
                    }
                    previous_operation = current_operation;
                    current_operation = Action.WRITE_POINT_A;
                    if (previous_operation != current_operation || first_time) {
                        telemetry.addData("Press a"," Now update Ax, Ay position of Rectangle");
                        telemetry.addData("              "," using the dPad");
                        telemetry.update();
                        first_time = false;
                        button_summary = false;
                        while (!gamepad1.a);  // Delay until a is released
                        while (gamepad1.a);
                    }

                } else if (gamepad1.b) {
                    // Specify Point Bx & By values
                    if (current_operation == Action.WRITE_POINT_B && previous_operation == Action.DO_NOTHING ) {
                        first_time = true;
                    }
                    previous_operation = current_operation;
                    current_operation = Action.WRITE_POINT_B;
                    if (previous_operation != current_operation || first_time) {
                        telemetry.addData("Press b"," Now update Bx, By position of Rectangle");
                        telemetry.addData("              "," using the dPad");
                        telemetry.update();
                        first_time = false;
                        button_summary = false;
                        while (!gamepad1.b);  // Delay until b is released
                        while (gamepad1.b);
                    }

                } else if (gamepad1.y) {
                    // Write out Point A and Point B values to rectangular.txt
                    previous_operation = current_operation;
                    current_operation = Action.WRITE_FILE;
                    telemetry.addData("Press y"," Now update rectangle.txt");
                    if (previous_operation != current_operation) {
                        telemetry.addData("Press y"," Now update rectangle.txt");
                        telemetry.update();
                        button_summary = false;
                        while (!gamepad1.y);  // Delay until y is released
                        while (gamepad1.y);
                    }
                } else if (gamepad1.x) {
                    // Write out Point A and Point B values to rectangular.txt, then exit program
                    current_operation = Action.WRITE_FILE_THEN_EXIT;
                    previous_operation = current_operation;
                    if (previous_operation != current_operation) {
                        telemetry.addData("Press x"," Now update rectangle.txt then exit");
                        telemetry.update();
                        button_summary = false;
                        while (!gamepad1.x);  // Delay until x is released
                        while (gamepad1.x);
                    }
                } else if (gamepad1.right_bumper) {
                    // gamepad1.right_bumper (right bumper button) is pressed
                    // Stop running the webcam pipeline class
                    // NOTE: stopping the stream from the camera early (before the end of the OpMode
                    //  when it will be automatically stopped for you) *IS* supported.
                    while (!gamepad1.right_bumper);  // Delay until the right_bumper is released
                    while (gamepad1.right_bumper);
                    if (webcam_pipeline_running) {
                        while (true) {
                            telemetry.addData("Press start button", "To CONFIRM stopping webcam");
                            telemetry.addData("Press back button", "To CANCEL stopping webcam");
                            telemetry.update();
                            if (gamepad1.start) {
                                previous_operation = current_operation;
                                current_operation = Action.STOP_WEBCAM_PIPELINE;
                                button_summary = false;
                                break;
                            } else if (gamepad1.back) {
                                //  Simply ignore cancelled request
                                previous_operation = current_operation;
                                current_operation = Action.DO_NOTHING;
                                break;
                            } else {
                                continue;  // re-prompt the user to confirm stop webcam operation
                            }
                        }
                    } else {
                        telemetry.addData("Press start button to acknowledge","");
                        telemetry.addData("","Webcam has already been stopped");
                        telemetry.update();
                        while (!gamepad1.start);  // Delay until the operation has been acknowledged
                    }
                } else {
                    // gamepad1.ps (logitech button) is pressed
                    // Re-display Button actions
                    first_time = true;
                    button_summary = true;
                    previous_operation = current_operation;
                    current_operation = Action.DO_NOTHING;
                    while (!gamepad1.ps);  // Delay until ps (logitech button) is released
                    while (gamepad1.ps);
                    continue;  // Cycle to the beginning of the while loop
                }
            }

            switch (current_operation) {
                case WRITE_POINT_A:
                    // Update Point A of the rectangle's position
                    // Use gamepad Dpad buttons to control the position of the rectangle
                    //   DpadLeft  button move the rectangle 10 pixels to the left
                    //   DpadRight  button move the rectangle 10 pixels to the right
                    //   DpadUp  button move the rectangle 10 pixels up
                    //   DpadDown  button move the rectangle 10 pixels down

                    if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
                        if (gamepad1.dpad_right) {
                            // Possible increase of rectangle's X coordinate
                            if (rectangle_opmode[0] + width_increment <= webcam_width) {
                                rectangle_opmode[0] += width_increment;
                            } else {
                                rectangle_opmode[0] = webcam_width;
                            }
                        } else if (gamepad1.dpad_left) {
                            // Possible decrease of rectangle's X coordinate
                            if (rectangle_opmode[0] - width_increment >= 0) {
                                rectangle_opmode[0] -= width_increment;
                            } else {
                                rectangle_opmode[0] = 0;
                            }
                        } else if (gamepad1.dpad_up) {
                            // Possible increase of rectangle's Y coordinate
                            if (rectangle_opmode[1] + height_increment <= webcam_height) {
                                rectangle_opmode[1] += height_increment;
                            } else {
                                rectangle_opmode[1] = webcam_height;
                            }
                        } else {
                            // Possible decrease of rectangle's Y coordinate
                            if (rectangle_opmode[1] - height_increment >= 0) {
                                rectangle_opmode[1] -= height_increment;
                            } else {
                                rectangle_opmode[1] = 0;
                            }
                        }
                    }
                    // Delay until all Dpad buttons have been released
                    while (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right);
                    break;

                case WRITE_POINT_B:
                    // Update Point B of the rectangle's position
                    // Use gamepad Dpad buttons to control the position of the rectangle
                    //  DpadLeft  button move the rectangle 10 pixels to the left
                    //  DpadRight  button move the rectangle 10 pixels to the right
                    //  DpadUp  button move the rectangle 10 pixels up
                    //  DpadDown  button move the rectangle 10 pixels down

                    if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
                        if (gamepad1.dpad_right) {
                            // Possible increase of rectangle's X coordinate
                            if (rectangle_opmode[2] + width_increment <= webcam_width) {
                                rectangle_opmode[2] += width_increment;
                            } else{
                                rectangle_opmode[2] = webcam_width;
                            }
                        } else if (gamepad1.dpad_left) {
                            // Possible decrease of rectangle's X coordinate
                            if (rectangle_opmode[2] - width_increment >= 0) {
                                rectangle_opmode[2] -= width_increment;
                            } else{
                                rectangle_opmode[2] = 0;
                            }

                        } else if (gamepad1.dpad_up) {
                            // Possible increase of rectangle's Y coordinate
                            if (rectangle_opmode[3] + height_increment <= webcam_height) {
                                rectangle_opmode[3] += height_increment;
                            } else{
                                rectangle_opmode[3] = webcam_height;
                            }
                        } else {
                            // Possible decrease of rectangle's Y coordinate
                            if (rectangle_opmode[3] - height_increment >= 0) {
                                rectangle_opmode[3] -= height_increment;
                            } else{
                                rectangle_opmode[3] = 0;
                            }
                        }
                    }
                    // Delay until all Dpad buttons have been released
                    while (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right);
                    break;

                case WRITE_FILE:
                    // Write rectangle.txt with the current positions of Point A and Point B

                case WRITE_FILE_THEN_EXIT:
                    // Write rectangle.txt with the current positions of Point A and Point B
                    // The enum Action.WRITE_FILE_THEN_EXIT will be used to exit the program at the end of the
                    //  main processing loop
                    valid_saved_rectangle_opmode_settings = false;
                    try {
                        FileWriter writer = new FileWriter(file_path, false);
                        BufferedWriter bufferedWriter = new BufferedWriter(writer);
                        for (i = 0; i < saved_rectangle_opmode.length; i++) {
                                saved_rectangle_opmode[i] = rectangle_opmode[i];
                                bufferedWriter.write(String.valueOf(saved_rectangle_opmode[i]));
                                bufferedWriter.newLine();
                        }
                        bufferedWriter.close();
                        writer.close();
                    } catch (IOException e) {
                        /*                e.printStackTrace();*/
                        telemetry.addData("File Write IOException",e);
                        telemetry.addData("Press back button to acknowledge","");
                        telemetry.update();
                        while (!gamepad1.back);
                    }
                    if (current_operation == Action.WRITE_FILE_THEN_EXIT) {
                        // It is necessary to restart (power cycle) the robot, for the webcam to work
                        telemetry.addData("Press start button to acknowledge","");
                        telemetry.addData("","Rectangular.txt has been written");
                        telemetry.addData("","Program will now close");
                        telemetry.addData("","You need to restart the robot");
                        current_operation = Action.STOP_PROGRAM;  // Set current operation to STOP_PROGRAM
                        previous_operation = Action.DO_NOTHING;
                    } else {
                        // current_operation == Action.WRITE_FILE
                        telemetry.addData("Press start button to acknowledge","");
                        telemetry.addData("","Rectangular.txt has been written");
                        current_operation = Action.WRITE_POINT_A;  // Reset current_operation to update Point A coordinates
                        previous_operation = Action.DO_NOTHING;
                    }
                    telemetry.update();
                    while (!gamepad1.start);  // Delay until the operation has been acknowledged
                    valid_saved_rectangle_opmode_settings = true;
                    break;

                case STOP_WEBCAM_PIPELINE:
                    /*
                     * IMPORTANT NOTE: calling stopStreaming() will indeed stop the stream of images
                     * from the camera (and, by extension, stop calling your vision pipeline). HOWEVER,
                     * if the reason you wish to stop the stream early is to switch use of the camera
                     * over to, say, Vuforia or TFOD, you will also need to call closeCameraDevice()
                     * (commented out below), because according to the Android Camera API documentation:
                     *         "Your application should only have one Camera object active at a time for
                     *          a particular hardware camera."
                     *
                     * NB: calling closeCameraDevice() will internally call stopStreaming() if applicable,
                     * but it doesn't hurt to call it anyway, if for no other reason than clarity.
                     *
                     * NB2: if you are stopping the camera stream to simply save some processing power
                     * (or battery power) for a short while when you do not need your vision pipeline,
                     * it is recommended to NOT call closeCameraDevice() as you will then need to re-open
                     * it the next time you wish to activate your vision pipeline, which can take a bit of
                     * time. Of course, this comment is irrelevant in light of the use case described in
                     * the above "important note".
                     */

                    webcam.stopStreaming();
                    webcam_pipeline_running = false;
                    current_operation = Action.WRITE_POINT_A;  // Reset current_operation to update Point A coordinates
                    previous_operation = Action.DO_NOTHING;
                    break;

                case STOP_PROGRAM:
                    // Actual termination of program takes place after the display of telemetry

                case DO_NOTHING:
                    // Just ignore
                    break;
            }

            /*
             * Send some stats to the telemetry
             */
/*            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());*/

            telemetry.addData("Pipeline Frame Count", webcam.getFrameCount());
            if (webcam_pipeline_running) {
                telemetry.addData("Pipeline status            "," Running");
            } else {
                telemetry.addData("Pipeline status            "," Stopped");
            }
            telemetry.addData("ROI         ", " Width x Height: %3d x %3d",
                    Math.abs(rectangle_opmode[2] - rectangle_opmode[0]), Math.abs(rectangle_opmode[3] - rectangle_opmode[1]));
            telemetry.addData("OpMode"," Ax: %3d, Ay: %3d, Bx: %3d, By: %3d",
                    rectangle_opmode[0], rectangle_opmode[1], rectangle_opmode[2], rectangle_opmode[3]);
            // IMPORTANT NOTE: Use getter method to allow code in the OpMode class to access the current rectangle settings
            //  that are defined in the Pipeline class
            telemetry.addData("Pipeline "," Ax: %3d, Ay: %3d, Bx: %3d, By: %3d",
                    pipeline.getRectangleFromPipeline(0), pipeline.getRectangleFromPipeline(1),
                    pipeline.getRectangleFromPipeline(2), pipeline.getRectangleFromPipeline(3));
            // IMPORTANT NOTE: Use getter method to allow code in the OpMode class to access the current rectangle settings
            //  that are defined in the Pipeline class
            telemetry.addData("Screen   "," Ax: %3d, Ay: %3d, Bx: %3d, By: %3d",
                    pipeline.getRectangleCoordinatesFromPipeline(0), pipeline.getRectangleCoordinatesFromPipeline(1),
                    pipeline.getRectangleCoordinatesFromPipeline(2), pipeline.getRectangleCoordinatesFromPipeline(3));
            telemetry.update();

            if (current_operation == Action.STOP_PROGRAM) {
                // Exit the program
                break;
            } else {
                // current == Action.DO_NOTHING)
                // Otherwise, ignore and cycle through the main processing loop
                sleep(100);
            }
        }
    }

    public boolean is_input_valid(String action) {
        boolean results;
        while (true) {
            telemetry.addData("Press start button", "To CONFIRM "+action);
            telemetry.addData("Press back button", "To CANCEL "+action);
            telemetry.update();
            if (gamepad1.start) {
                results =  true;
                break;
            } else if (gamepad1.back) {
                results = false;
                break;
            } else {
                
            }
        }
        return results;
    }

    public void setup_ROI (String action, String success_message1, String success_message2, int a_x, int a_y, int b_x, int b_y) {
        if (is_input_valid(action)) {
            valid_input = true;  // Response confirmed, continue processing
        } else {
            valid_input = false;  // Ask the question again
            return;
        }
        rectangle_opmode[0] = a_x;    // Ax
        rectangle_opmode[1] = a_y;    // Ay
        rectangle_opmode[2] = b_x;    // Bx
        rectangle_opmode[3] = b_y;    // By

        // Copy the rectangle setting to the saved rectangle settings
        for (i = 0; i < saved_rectangle_opmode.length; i++) {
            saved_rectangle_opmode[i] = rectangle_opmode[i];
        }
        telemetry.addData(success_message1, "");
        telemetry.addData(success_message2, "");
        return;

    }

    // IMPORTANT NOTE: Define getter method to allow code in the Pipeline class to access
    //  information about the rectangle that is stored in OpMode class
    public int getRectangleFromOpmode(int index) {
        return rectangle_opmode[index];
    }

    // IMPORTANT NOTE: Define getter method to allow code in the Pipeline class to access
    //  information about the rectangle that has been saved in rectangle.txt
    public int getSavedRectangleFromOpmode(int index) {
        if (valid_saved_rectangle_opmode_settings) {
            return saved_rectangle_opmode[index];  // Return saved position of rectangle
        } else {
            return -1;  // Return -1  to indicate saved rectangle position is not valid (being updated)
        }
    }


// *************************************************************************************************
//
//                                      SamplePipeline class
//                (For purposes of discussion, this is also called the "Pipeline" class)
//
// *************************************************************************************************

    /*
     * An example image processing pipeline to be run upon receipt of each frame from the camera.
     * Note that the processFrame() method is called serially from the frame worker thread -
     * that is, a new camera frame will not come in while you're still processing a previous one.
     * In other words, the processFrame() method will never be called multiple times simultaneously.
     *
     * However, the rendering of your processed image to the viewport is done in parallel to the
     * frame worker thread. That is, the amount of time it takes to render the image to the
     * viewport does NOT impact the amount of frames per second that your pipeline can process.
     *
     * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode. It is invoked on the
     * frame worker thread. This should not be a problem in the vast majority of cases. However,
     * if you're doing something weird where you do need it synchronized with your OpMode,
     * then you will need to account for that accordingly.
     */
    class SamplePipeline extends OpenCvPipeline
    {
        boolean viewportPaused;
        //IMPORTANT NOTE: Allows information defined in the Pipeline class to be accessed from the OpMode class
        //Declare instance variables that support the sharing of information between the two classes
        /*
         * rectangle_pipeline[0] / rectangle_coordinates_pipline[0] - Point A, X
         * rectangle_pipeline[1] / rectangle_coordinates_pipline[1] - Point A, Y
         * rectangle_pipeline[2] / rectangle_coordinates_pipline[2] - Point B, X
         * rectangle_pipeline[3] / rectangle_coordinates_pipline[3] - Point B, Y
         */
        int [] rectangle_pipeline = {0, 0, 0, 0};
        int [] rectangle_coordinates_pipline  = {0, 0, 0, 0};
        public int i;
        public boolean initialize_rectangle_coordinates_pipline = false;

        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
             * will only dereference to the same image for the duration of this particular
             * invocation of this method. That is, if for some reason you'd like to save a copy
             * of this particular frame for later use, you will need to either clone it or copy
             * it to another Mat.
             */

            // On initialization, read the coordinates for the viewing rectangle from rectangle.txt
            if (!initialize_rectangle_coordinates_pipline) {
                file_dir = Environment.getExternalStorageDirectory();
                file_path = new File(file_dir, file_name);
                initialize_rectangle_coordinates_pipline = true;
                if (!file_path.exists()) {
                    // Rectangle.txt does not exist set Point A and Point B rectangle coordinates to zero
                    for (i = 0; i < rectangle_coordinates_pipline.length; i++) {
                        rectangle_coordinates_pipline[i] = 0;
                    }
                } else {
                    // Rectangle.txt exists, read file to get rectangle coordinates
                    try {
                        FileReader reader = new FileReader(file_path);
                        BufferedReader bufferedReader = new BufferedReader(reader);
                        i = 0;
                        while ((line_of_text = bufferedReader.readLine()) != null) {
                            if (i <= 3) {
                                rectangle_coordinates_pipline[i] = Integer.parseInt(line_of_text);
                            } else {
                                break;
                            }
                            i++;
                        }
                        bufferedReader.close();
                        reader.close();
                    } catch (IOException e) {
                        telemetry.addData("File Read IOException in Pipline", e);
                        telemetry.addData("Press back button to acknowledge", "");
                        telemetry.update();
                        while (!gamepad1.back) ;
                    }
                }
            }

            // IMPORTANT NOTE: Use getter method to allow code in the Pipeline class to access the current rectangle settings
            //  that are defined in the OpMode class
            for (i = 0; i < rectangle_coordinates_pipline.length; i++) {
                rectangle_pipeline[i] = getRectangleFromOpmode(i);
                if (getSavedRectangleFromOpmode(i) != -1) {
                    // Current saved rectangle settings are valid,
                    //  update the current rectangle settings that are used to draw the rectangle
                    //  and also update the coordinates of viewing rectangle
                    rectangle_coordinates_pipline[i] = getSavedRectangleFromOpmode(i);
                } else {
                    // Current saved rectangle settings are not valid,
                    //  don't update the current rectangle settings that are used to draw the rectangle
                }
            }

            //
            // Draw the viewing rectangle with a small frame
            //
            Imgproc.rectangle(
                    input,
                    new Point(
                            rectangle_coordinates_pipline[0],
                            rectangle_coordinates_pipline[1]),
                    new Point(
                            rectangle_coordinates_pipline[2],
                            rectangle_coordinates_pipline[3]),
                    new Scalar(0, 255, 0), 4);
            //Change to allow information from the Pipeline class to be accessed from the OpMode class
            //Update the current position of the rectangle

            /*
             * NOTE: to see how to get data from your pipeline to your OpMode as well as how
             * to change which stage of the pipeline is rendered to the viewport when it is
             * tapped, please see {@link PipelineStageSwitchingExample}
             */

            return input;
        }

        // IMPORTANT NOTE: Define getter method to allow code in the OpMode class to access information about the rectangle
        //  that is stored in Pipeline class
        public int getRectangleFromPipeline(int index) {

            return rectangle_pipeline[index];
        }

        // IMPORTANT NOTE: Define getter method to allow code in the OpMode class to access information about the saved rectangle
        //  that is stored in Pipeline class
        public int getRectangleCoordinatesFromPipeline(int index) {
            return rectangle_coordinates_pipline[index];
        }

        @Override
        public void onViewportTapped()
        {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */

            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }
        }
    }
}