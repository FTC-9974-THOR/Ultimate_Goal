
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.RobotLog;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


import java.util.concurrent.atomic.AtomicBoolean;

// where did this come from? and why?
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

// To do 12/13/20: determine a value for cbThreshold; adjust anchor points, region widths, and region heights
//12/14/20 Added Saturation!

public class StackVisionPipeline extends OpenCvPipeline {

    private static final String TAG = "StackVisionPipeline";

    public enum StackHeight{
        ZERO,
        ONE,
        FOUR
    }
    static final Scalar GREEN = new Scalar(0, 255, 0);
    static final Scalar BLUE = new Scalar(0,0,255);

    //StackHeight height = StackHeight.ZERO;
    StackHeight height = null;

    // since I had to change the streaming resolution, I had to scale each point, width, and height
    // to match. each point goes to the same place in the image; the image is just at a lower
    // resolution.
    private static final double X_SCALE_FACTOR = 640.0 / 1280.0;
    private static final double Y_SCALE_FACTOR = 480.0 / 720.0;

    /* There are two regions: upperRegion and lowerRegion. They have the same width, but not the same height.
     */
    static final Point UPPER_REGION_ANCHOR_POINT = new Point(500 * X_SCALE_FACTOR,85 * Y_SCALE_FACTOR);//was 109, 98
    static final Point LOWER_REGION_ANCHOR_POINT = new Point(500 * X_SCALE_FACTOR, 140 * Y_SCALE_FACTOR);

    static final int REGION_WIDTH = (int) (10 * X_SCALE_FACTOR);
    static final int UPPER_REGION_HEIGHT = (int) (25 * Y_SCALE_FACTOR);
    static final int LOWER_REGION_HEIGHT = (int) (10 * Y_SCALE_FACTOR);

    Point upperRegion_pointA = new Point(UPPER_REGION_ANCHOR_POINT.x, UPPER_REGION_ANCHOR_POINT.y);
    Point upperRegion_pointB = new Point(UPPER_REGION_ANCHOR_POINT.x + REGION_WIDTH, UPPER_REGION_ANCHOR_POINT.y + UPPER_REGION_HEIGHT);

    Point lowerRegion_pointA = new Point(LOWER_REGION_ANCHOR_POINT.x, LOWER_REGION_ANCHOR_POINT.y);
    Point lowerRegion_pointB = new Point(LOWER_REGION_ANCHOR_POINT.x + REGION_WIDTH, LOWER_REGION_ANCHOR_POINT.y + LOWER_REGION_HEIGHT);

    Mat upper_region_H, lower_region_H;
    Mat upper_region_S, lower_region_S;
    Mat HSV = new Mat();
    Mat Hue = new Mat();
    Mat Saturation = new Mat();

    double upper_region_H_average, lower_region_H_average, upper_region_S_average,lower_region_S_average;

    // variable to keep track of if a call to processFrame() has completed.
    // since processFrame() seems to be called from another thread, this variable is atomic to make
    // it thread safe.
    private final AtomicBoolean hasCompletedAnalysis;

    public StackVisionPipeline() {
        // initialize hasCompletedAnalysis
        hasCompletedAnalysis = new AtomicBoolean(false);
    }

    void inputToH(Mat input)
    {
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
        Core.extractChannel(HSV, Hue, 0);
    }

    void inputToS(Mat input)
    {
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
        Core.extractChannel(HSV, Saturation,1);
    }

    @Override
    public void init(Mat firstFrame)
    {
        /*
         * We need to call this in order to make sure the 'Cb'
         * object is initialized, so that the submats we make
         * will still be linked to it on subsequent frames. (If
         * the object were to only be initialized in processFrame,
         * then the submats would become delinked because the backing
         * buffer would be re-allocated the first time a real frame
         * was crunched)
         */
        inputToH(firstFrame);
        inputToS(firstFrame);

        /*
         * Submats are a persistent reference to a region of the parent
         * buffer. Any changes to the child affect the parent, and the
         * reverse also holds true.
         */
        upper_region_H = Hue.submat(new Rect(upperRegion_pointA, upperRegion_pointB));
        lower_region_H = Hue.submat(new Rect(lowerRegion_pointA, lowerRegion_pointB));

        upper_region_S = Saturation.submat(new Rect (upperRegion_pointA, upperRegion_pointB));
        lower_region_S = Saturation.submat(new Rect (lowerRegion_pointA, lowerRegion_pointB));

    }

    @Override
    public Mat processFrame(Mat input)
    {
        RobotLog.ii(TAG, "processFrame()");
        inputToH(input);
        inputToS(input);

        /*
         * Compute the average pixel value of each submat region. We're
         * taking the average of a single channel buffer, so the value
         * we need is at index 0. We could have also taken the average
         * pixel value of the 3-channel image, and referenced the value
         * at index 2 here.
         */
        upper_region_H_average = (int) Core.mean(upper_region_H).val[0];
        lower_region_H_average = (int) Core.mean(lower_region_H).val[0];

        upper_region_S_average = (int) Core.mean(upper_region_S).val[0];//not sure if the 0 is right
        lower_region_S_average = (int) Core.mean(lower_region_S).val[0];//not sure on this either

        double hMinimum = 10;
        double hMaximum = 20;//this was 29

        double sMinimum = 150;//this was 50
        double sMaximum = 255;//this was 255
        /*
         * Draw rectangles showing the three regions on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                upperRegion_pointA, // First point which defines the rectangle
                upperRegion_pointB, // Second point which defines the rectangle
                GREEN, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        Imgproc.rectangle(
                input, // Buffer to draw on
                lowerRegion_pointA, // First point which defines the rectangle
                lowerRegion_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2);

        /*
         * Now that we found the max, we actually need to go and
         * figure out which sample region that value was from
         */
        if (upper_region_H_average >= hMinimum && upper_region_H_average <= hMaximum && upper_region_S_average >= sMinimum && upper_region_S_average <= sMaximum){
            height = StackHeight.FOUR;
        } else if (lower_region_H_average >= hMinimum && lower_region_H_average <= hMaximum && lower_region_S_average >= sMinimum && lower_region_S_average <= sMaximum){
            height = StackHeight.ONE;
        } else {
            height = StackHeight.ZERO;
        }

        // these were referencing a static import to some part of the SDK. apparently the variable
        // they reference (telemetry) is never initialized so it was throwing NullPointerExceptions.
        /*telemetry.addData("[Pattern]", height);
        telemetry.addData("UpperRegionHAverage: ", upper_region_H_average);
        telemetry.addData("LowerRegionHAverage: ", lower_region_H_average);
        telemetry.addData("UpperRegionSAverage: ", upper_region_S_average);
        telemetry.addData("LowerRegionSAverage: ", lower_region_S_average);
        telemetry.update();*/

        // so I printed the same information to the RobotLog.
        RobotLog.dd(TAG, "Pattern: %s", height.toString());
        RobotLog.vv(TAG, "UpperRegionHAverage: %.5f", upper_region_H_average);
        RobotLog.vv(TAG, "LowerRegionHAverage: %.5f", lower_region_H_average);
        RobotLog.vv(TAG, "UpperRegionSAverage: %.5f", upper_region_S_average);
        RobotLog.vv(TAG, "LowerRegionSAverage: %.5f", lower_region_S_average);

        // set hasCompletedAnalysis to true to indicate that the pipeline has processed at least one
        // frame
        hasCompletedAnalysis.set(true);

        /*
         * Render the 'input' buffer to the viewport. But note this is not
         * simply rendering the raw camera feed, because we called functions
         * to add some annotations to this buffer earlier up.
         */
        return input;
    }

    public boolean hasAnalysis() {
        return hasCompletedAnalysis.get();
    }

    public StackHeight getAnalysis()
    {
        return height;
    }
}

