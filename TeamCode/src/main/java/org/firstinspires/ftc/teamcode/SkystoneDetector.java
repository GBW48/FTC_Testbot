/**
 * Acknowledgements
 * This program is based on the SkystoneCVTutorial which was developed by wolfcorpftc
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


import static android.os.SystemClock.sleep;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


class SkystoneDetector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location {
        LEFT,
        RIGHT,
        NOT_FOUND
    }
    private Location location;
    static private int horizontal_offset = 10;

    static final Rect LEFT_ROI = new Rect(
            new Point(92 + horizontal_offset, 170),
            new Point(152 - horizontal_offset, 220));
    static final Rect RIGHT_ROI = new Rect(
            new Point(153 + horizontal_offset, 170),
            new Point(213 - horizontal_offset, 220));

    static double PERCENT_COLOR_THRESHOLD = 0.4;
    
    public SkystoneDetector(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(16, 110, 165);
        Scalar highHSV = new Scalar(39, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

/*        left.release();  // Need to be released after display of left and right raw telemetry data
        right.release();*/

        telemetry.addData("Left raw value        ", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Right raw value      ", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Treshold percentage ", PERCENT_COLOR_THRESHOLD * 100 + "%");
        telemetry.addData("Left percentage     ", Math.round(leftValue * 100) + "%");
        telemetry.addData("Right percentage   ", Math.round(rightValue * 100) + "%");

        left.release();
        right.release();

        boolean stoneLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean stoneRight = rightValue > PERCENT_COLOR_THRESHOLD;


/*        if (stoneLeft && stoneRight) {
            location = Location.NOT_FOUND;
            telemetry.addData("Skystone Location", "not found");
        }
        else if (stoneLeft) {
            location = Location.RIGHT;
            telemetry.addData("Skystone Location", "right");
        }
        else {
            location = Location.LEFT;
            telemetry.addData("Skystone Location", "left");
        }*/

        // Improved version of code to detect if a right or left Skystone has been detected
        //  Note: the code assumes, that there is only one Skystone which is next to a normal stone
        //  (two Skystones will not be detected)
        if (stoneLeft && stoneRight || !stoneLeft  && !stoneRight) {
            location = Location.NOT_FOUND;
            telemetry.addData("Skystone Location", "not found");
        } else if (!stoneRight && stoneLeft) {
            location = Location.RIGHT;
            telemetry.addData("Skystone Location", "right");
        } else {
            /* !stoneLeft && stoneRight) */
            location = Location.LEFT;
            telemetry.addData("Skystone Location", "left");
        }

        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorStone = new Scalar(255, 0, 0);
        Scalar colorSkystone = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT? colorSkystone:colorStone);
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? colorSkystone:colorStone);

        return mat;
    }

    public Location getLocation() {
        return location;
    }
}