/**
 * Acknowledgements
 * This program is based on a TensorFlow Lite Object Detection Program developed and written by Ed C. Epp
 *      The code can be found at:
 *          https://github.com/edcepp/FTCEppTensorCode/blob/master/FTCEppTensorCode/TestTensorFlowObjectDetectionWebcamEpp.java
*       Information about the implementation of the program can be found at:
 *          https://github.com/edcepp/FTCEppTensorCode/tree/master/FTCEppTensorCode
 *      In addition, a related video tutorial can be found at:
 *          https://www.youtube.com/watch?v=Cd2PYhapyvw&t=752s
 */

/* Copyright (c) 2019 FIRST. All rights reserved.
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
 *
 * Modifications by
 * Ed C. Epp 10-2021
 *
 * Test TensorFlow for Object Detection using a Webcam 
 *
 * The following actions are demostrated
 *   Initialize a webcam using using Vuforial libraries
 *   Initialize the TensorFlow Object Detector Engine called "Tfod" and
 *       Load in a TensorFlow Lite model
 *   Wait for the user to press start
 *   For the next 30 seconds or until stop is pressed repeat the following
 *      Get a list of recognized objects - the list may be empty
 *         Name each recognized object 
 *   
 ******************************************************************************************/

// package org.firstinspires.ftc.teamcode;
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Freight Frenzy game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

/*@TeleOp(name="Pushbot: Teleop Custom TensorFlow test with webcam (Epp)", group="Pushbot")
// @Disabled
public class TensorFlowLiteAuto_FindBallCubeDuck_LinearPushbot_TeleopCustomObjectDetectionWebcamEppLinear extends LinearOpMode*/

@Autonomous(name="TensorFlowLite: Find Ball, Cube, Duck", group="Vision")
// @Disabled

public class TensorFlowLiteAuto_FindBallCubeDuck_Linear extends LinearOpMode
{

/*    Testbot_Hardware robot          = new Testbot_Hardware();   // Enable to use Testbot's hardware definition*/

    /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
   * the following 4 detectable objects
   *  0: Ball,
   *  1: Cube,
   *  2: Duck,
   *  3: Marker (duck location tape marker)
   *  Two additional model assets are available which only contain a subset of the objects:
   *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
   *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
   */

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
      "Ball",
      "Cube",
      "Duck",
      "Marker"
    };

    enum AngleUnit {
        DEGREES,
        RADIANS
    }

    int image_width;
    int image_height;
    double left;
    double right;
    double top;
    double bottom;
    double width;
    double height;
    double x_coordinate;
    double y_coordinate;
    double image_height_double;
    public float minResultConfidence = 0.7f;


    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. This licence belongs to 
     * Dr. Edward C. Epp. Please do not use it. The string below with which     
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */

    // Don't use this Vuforia Key, please get your own key
    private static final String VUFORIA_KEY = "AX2vfaD/////AAABmSKKqRQD9EkSgISOFzC3odhaP9U5eC18G4bM4Zc9MoILo8gUkEfyfZAolyjZVIDqD5q/BDjWUzzwPw7UIgOQRAczYzK+fZIo2nQlhm5w5Calfa9h6/XEQvLbJNVdwgFOfg/otLRMR6o+7Jsk1AhrBufUXsha57SM18/qnpTgn9T6931R1AXt2bLeM0WoNTugCNk51HJaD6jJNYrDSiEvOL/TaST1LZTKN/t4Lfh9nUnmNYYrbKX2ezQtBiR72T1g8CfTHcz+vk34khcwRGJ1aGF5JkXvK1IbrhREVH09Hyd0wzpi06VfiJckojP9Zpjk3p6me9bNW2dksknhv9/ULFkYZruXpKg7dhyGSrtCWUwA";
    // Don't use this Vuforia Key, please get your own key

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    
    /***************************** runOpMode ****************************
     * Execution begins here
     * Initialize the TensorFlow Object Detection engine.
    ********************************************************************/
    @Override
    public void runOpMode() {

/*        robot.init(hardwareMap);  // Enable to use Testbot's hardware definition*/

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            // feature disabled
            // tfod.setZoom(2.5, 16.0/9.0);
        }

        /** Wait for the game to begin */

        // Send telemetry message to indicate robot is ready to go:
        telemetry.addData("On phone press '3 vertical dots' then 'Camera Stream'","");
        telemetry.addData("          ", "To display objects");
        telemetry.addData("Or press Play","");
        telemetry.addData("          ", "To identify objects");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("Number of Object(s) Detected", updatedRecognitions.size());
                        telemetry.addData("Minimum Confidence                ", minResultConfidence);
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            /*
                            Interface Recognition - Method Summary
                            java.lang.String getLabel()
                            Returns the label of the detected object.
                            float getConfidence()
                            Returns the confidence of the detection.
                            float getLeft()
                            Returns the left coordinate of the rectangle bounding the detected object.
                            float getRight()
                            Returns the right coordinate of the rectangle bounding the detected object.
                            float getTop()
                            Returns the top coordinate of the rectangle bounding the detected object.
                            float getBottom()
                            Returns the bottom coordinate of the rectangle bounding the detected object.
                            float getWidth()
                            Returns the width of the rectangle bounding the detected object.
                            float getHeight()
                            Returns the height of the rectangle bounding the detected object.
                            int getImageWidth()
                            Returns the width of the entire image.
                            int getImageHeight()
                            Returns the height of the entire image
                            double estimateAngleToObject(AngleUnit angleUnit)
                            Returns an estimation of the horizontal angle to the detected object.
                            */

                            telemetry.addData(String.format("Detected Object     [%d]", i), recognition.getLabel());
                            telemetry.addData(String.format("    [%d] Confidence         ", i), "%2.1f",recognition.getConfidence());
                            left = recognition.getLeft();
                            top = recognition.getTop();
                            telemetry.addData(String.format("    [%d] Left,Top               ", i), "%.0f , %.0f", left, top);
                            right = recognition.getRight();
                            bottom = recognition.getBottom();
                            telemetry.addData(String.format("    [%d] Right,Bottom      ", i), "%.0f , %.0f", right, bottom);
                            width = recognition.getWidth();
                            height = recognition.getHeight();
                            telemetry.addData(String.format("    [%d] Rectangle_Width,Rectangle_Height  ", i), "%.0f , %.0f", width, height);
                            image_width = recognition.getImageWidth();
                            image_height = recognition.getImageHeight();
                            image_height_double = Double.valueOf(image_height);
                            telemetry.addData(String.format("    [%d] Image_Width,Image_Height                ", i), "%3d , %3d", image_width, image_height);
                            // Calculate center coordinates of detected object (based on a screen size of X pixels wide and Y pixels high)
                            x_coordinate = left + .5 * (width);
                            y_coordinate = image_height_double - bottom + .5 * (height);  //Note top dimension starts at 0
                            telemetry.addData(String.format("    [%d] Object_X,Object_Y                                 ", i), "%.0f , %.0f", x_coordinate, y_coordinate);
                            telemetry.addData(String.format("    [%d] Estimated horizontal angle to object", i), "%2.0f",
                                    recognition.estimateAngleToObject(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES));
                            i++;
                        }
                        telemetry.update();
                    }
                }
            }
        }
    }

    /***************************** initVuforia ***************************
     * Initialize the Vuforia localization engine.
     ********************************************************************/
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /***************************** initTfod ******************************
     * Initialize the TensorFlow Object Detection engine.
     ********************************************************************/
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
/*        tfodParameters.minResultConfidence = 0.8f;*/
/*        tfodParameters.minResultConfidence = 0.7f;*/
        tfodParameters.minResultConfidence = minResultConfidence;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}