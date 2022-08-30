/**
 * Acknowledgements
 * This program is based in part, on the SkystoneCVTutorial which was developed by wolfcorpftc
 *      The code can be found at:
 *          https://github.com/wolfcorpftc/SkystoneCVTutorial
 *      In addition a video tutorial can be found at:
 *          https://www.youtube.com/watch?v=JO7dqzJi8lw
 *
 *In addition, this program uses EasyOpenCV which was developed by OpenFTC
 *      Information about EasyOpenCV can be found at:
 *          https://github.com/OpenFTC/EasyOpenCV#installation-instructions-android-studio
 */

/*package org.wolfcorp.cv.tutorial;*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="OpenCV: Find Skystones Enhanced", group="Vision")
//@Disabled

public class OpenCVAuto_FindSkystonesEnhanced_Linear extends LinearOpMode {

/*    Testbot_Hardware robot          = new Testbot_Hardware();

// *************************************************************************************************
// Begin phoneCam code
/*    OpenCvCamera phoneCam;
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance()
                .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        SkystoneDetector detector = new SkystoneDetector(telemetry);
        phoneCam.setPipeline(detector);
        phoneCam.openCameraDeviceAsync(
                () -> phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
        );
        phoneCam.openCameraDeviceAsync(
                () -> phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
        );*/
// End phoneCam code
// *************************************************************************************************

// *************************************************************************************************
// Begin webCam code
    OpenCvWebcam webcam;
    @Override
    public void runOpMode() throws InterruptedException {

/*    robot.init(hardwareMap);  // Enable to use Testbot's hardware definition*/

    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

    SkystoneDetectorEnhanced detector = new SkystoneDetectorEnhanced(telemetry);
    webcam.setPipeline(detector);

    // OR...  Do Not Activate the Camera Monitor View
    //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

    /*
     * Specify the image processing pipeline we wish to invoke upon receipt
     * of a frame from the camera. Note that switching pipelines on-the-fly
     * (while a streaming session is in flight) *IS* supported.
     */

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
            webcam.showFpsMeterOnViewport(false);  // When using external monitor, don't turn-on FPS overlay
            webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        }

        @Override
        public void onError(int errorCode)
        {
            /*
             * This will be called if the camera could not be opened
             */
        }
    });

// End webCam code
// *************************************************************************************************

        waitForStart();

        switch (detector.getLocation()) {
            case LEFT:
                // ...
                break;
            case RIGHT:
                // ...
                break;
            case LEFT_AND_RIGHT:
                // ...
                break;
            case NOT_FOUND:
                // ...
        }
        webcam.stopStreaming();
    }
}