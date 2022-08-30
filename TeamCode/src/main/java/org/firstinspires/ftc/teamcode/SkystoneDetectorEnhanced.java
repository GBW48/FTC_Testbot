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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

class SkystoneDetectorEnhanced extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location {
        LEFT,
        RIGHT,
        LEFT_AND_RIGHT,
        NOT_FOUND
    }

    private Location location;
    final static private int horizontal_offset = 10;
    final static private int top_vertical_offset = 20;
    final static private int bottom_vertical_offset = 55;

    static final Rect LEFT_ROI = new Rect(
        new Point(92 + horizontal_offset, 170),
        new Point(152 - horizontal_offset, 220));
    static final Rect LEFT_TOP_ROI = new Rect(
        new Point(92 + horizontal_offset, 170 - top_vertical_offset),
        new Point(152 - horizontal_offset, 220 - bottom_vertical_offset));
    static final Rect RIGHT_ROI = new Rect(
        new Point(153 + horizontal_offset, 170),
        new Point(213 - horizontal_offset, 220));
    static final Rect RIGHT_TOP_ROI = new Rect(
        new Point(153 + horizontal_offset, 170 - top_vertical_offset),
        new Point(213 - horizontal_offset, 220 - bottom_vertical_offset));

    static double PERCENT_COLOR_THRESHOLD = 0.4;
    public SkystoneDetectorEnhanced(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(16, 110, 165);
        Scalar highHSV = new Scalar(39, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat left_top = mat.submat(LEFT_TOP_ROI);
        Mat right = mat.submat(RIGHT_ROI);
        Mat right_top = mat.submat(RIGHT_TOP_ROI);

        telemetry.addData("Left raw value             ", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Left Top raw value     ", (int) Core.sumElems(left_top).val[0]);
        telemetry.addData("Right raw value           ", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Right Top raw value   ", (int) Core.sumElems(right_top).val[0]);

        double leftAverage = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double lefttopAverage = Core.sumElems(left_top).val[0] / LEFT_TOP_ROI.area() / 255;
        double rightAverage = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;
        double righttopAverage = Core.sumElems(right_top).val[0] / RIGHT_TOP_ROI.area() / 255;

        telemetry.addData("Treshold  percentage", PERCENT_COLOR_THRESHOLD * 100 + "%");
        telemetry.addData("Left           percentage", Math.round(leftAverage * 100) + "%");
        telemetry.addData("Left Top   percentage", Math.round(lefttopAverage * 100) + "%");
        telemetry.addData("Right         percentage", Math.round(rightAverage * 100) + "%");
        telemetry.addData("Right Top percentage", Math.round(righttopAverage * 100) + "%");

        left.release();
        left_top.release();
        right.release();
        right_top.release();

        boolean stoneLeft = (leftAverage <= PERCENT_COLOR_THRESHOLD) && (lefttopAverage > PERCENT_COLOR_THRESHOLD);
        boolean stoneRight = (rightAverage <= PERCENT_COLOR_THRESHOLD) && (righttopAverage > PERCENT_COLOR_THRESHOLD);

        if (!stoneLeft && !stoneRight) {
            location = Location.NOT_FOUND;
            telemetry.addData("Skystone Location", "none found");
        } else if  (stoneLeft && stoneRight) {
            location = Location.LEFT_AND_RIGHT;
            telemetry.addData("Skystone Location", "found on left");
            telemetry.addData("Skystone Location", "found on right");
        }else if (stoneLeft) {
            location = Location.LEFT;
            telemetry.addData("Skystone Location", "found on left");
        } else {
            // stoneRight
            location = Location.RIGHT;
            telemetry.addData("Skystone Location", "found on right");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorStone = new Scalar(255, 0, 0);
        Scalar colorSkystone = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT? colorSkystone:colorStone);
        Imgproc.rectangle(mat, LEFT_TOP_ROI, location == Location.LEFT? colorSkystone:colorStone);
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? colorSkystone:colorStone);
        Imgproc.rectangle(mat, RIGHT_TOP_ROI, location == Location.RIGHT? colorSkystone:colorStone);

        return mat;
    }

    public Location getLocation() {
        return location;
    }
}