package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class RedTeamElementDeterminationPipeline extends OpenCvPipeline
{
    /*
     * An enum to define the TeamElement position
     */
    public enum TeamElementPosition
    {
        LEFT,
        CENTER,
        RIGHT
    }

    /*
     * Some color constants
     */
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);
    static final Scalar RED = new Scalar(255, 0, 0);

    /*
     * The core values which define the location and size of the sample regions
     */
//    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(0,80);
//    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(160,80);
//    static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(560,80);
//    static final Point REGION1_BOTTOMRIGHT_ANCHOR_POINT = new Point(160,320);
//    static final Point REGION2_BOTTOMRIGHT_ANCHOR_POINT = new Point(480,160);
//    static final Point REGION3_BOTTOMRIGHT_ANCHOR_POINT = new Point(640,320);
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(640,400);
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(480,400);
    static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(80,400);
    static final Point REGION1_BOTTOMRIGHT_ANCHOR_POINT = new Point(480,160);
    static final Point REGION2_BOTTOMRIGHT_ANCHOR_POINT = new Point(160,320);
    static final Point REGION3_BOTTOMRIGHT_ANCHOR_POINT = new Point(0,160);

    /*
     * Points which actually define the sample region rectangles, derived from above values
     *
     * Example of how points A and B work to define a rectangle
     *
     *   ------------------------------------
     *   | (0,0) Point A                    |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                  Point B (70,50) |
     *   ------------------------------------
     *
     */

    /*
     * Working variables
     */
    Mat region1_Cb, region2_Cb, region3_Cb;
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    int avg1, avg2, avg3;

    // Volatile since accessed by OpMode thread w/o synchronization
    private volatile TeamElementPosition position = TeamElementPosition.LEFT;

    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Cb channel to the 'Cb' variable
     */
    void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 1);
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
        inputToCb(firstFrame);

        /*
         * Submats are a persistent reference to a region of the parent
         * buffer. Any changes to the child affect the parent, and the
         * reverse also holds true.
         */
        region1_Cb = Cb.submat(new Rect(REGION1_TOPLEFT_ANCHOR_POINT, REGION1_BOTTOMRIGHT_ANCHOR_POINT));
        region2_Cb = Cb.submat(new Rect(REGION2_TOPLEFT_ANCHOR_POINT, REGION2_BOTTOMRIGHT_ANCHOR_POINT));
        region3_Cb = Cb.submat(new Rect(REGION3_TOPLEFT_ANCHOR_POINT, REGION3_BOTTOMRIGHT_ANCHOR_POINT));
    }

    @Override
    public Mat processFrame(Mat input)
    {
        /*
         * Overview of what we're doing:
         *
         * We first convert to YCrCb color space, from RGB color space.
         * Why do we do this? Well, in the RGB color space, chroma and
         * luma are intertwined. In YCrCb, chroma and luma are separated.
         * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
         * are Y, the luma channel (which essentially just a B&W image), the
         * Cr channel, which records the difference from red, and the Cb channel,
         * which records the difference from blue. Because chroma and luma are
         * not related in YCrCb, vision code written to look for certain values
         * in the Cr/Cb channels will not be severely affected by differing
         * light intensity, since that difference would most likely just be
         * reflected in the Y channel.
         *
         * After we've converted to YCrCb, we extract just the 2nd channel, the
         * Cb channel. We do this because stones are bright yellow and contrast
         * STRONGLY on the Cb channel against everything else, including TeamElements
         * (because TeamElements have a black label).
         *
         * We then take the average pixel value of 3 different regions on that Cb
         * channel, one positioned over each stone. The brightest of the 3 regions
         * is where we assume the TeamElement to be, since the normal stones show up
         * extremely darkly.
         *
         * We also draw rectangles on the screen showing where the sample regions
         * are, as well as drawing a solid rectangle over top the sample region
         * we believe is on top of the TeamElement.
         *
         * In order for this whole process to work correctly, each sample region
         * should be positioned in the center of each of the first 3 stones, and
         * be small enough such that only the stone is sampled, and not any of the
         * surroundings.
         */

        /*
         * Get the Cb channel of the input frame after conversion to YCrCb
         */
        inputToCb(input);

        /*
         * Compute the average pixel value of each submat region. We're
         * taking the average of a single channel buffer, so the value
         * we need is at index 0. We could have also taken the average
         * pixel value of the 3-channel image, and referenced the value
         * at index 2 here.
         */
        avg1 = (int) Core.mean(region1_Cb).val[0];
        avg2 = (int) Core.mean(region2_Cb).val[0];
        avg3 = (int) Core.mean(region3_Cb).val[0];

        /*
         * Draw a rectangle showing sample region 1 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                REGION1_TOPLEFT_ANCHOR_POINT, // First point which defines the rectangle
                REGION1_BOTTOMRIGHT_ANCHOR_POINT, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        /*
         * Draw a rectangle showing sample region 2 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                REGION2_TOPLEFT_ANCHOR_POINT, // First point which defines the rectangle
                REGION2_BOTTOMRIGHT_ANCHOR_POINT, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        /*
         * Draw a rectangle showing sample region 3 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                REGION3_TOPLEFT_ANCHOR_POINT, // First point which defines the rectangle
                REGION3_BOTTOMRIGHT_ANCHOR_POINT, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines


        /*
         * Find the max of the 3 averages
         */
        int maxOneTwo = Math.max(avg1, avg2);
        int max = Math.max(maxOneTwo, avg3);

        /*
         * Now that we found the max, we actually need to go and
         * figure out which sample region that value was from
         */
        if(max == avg1) // Was it from region 1?
        {
            position = TeamElementPosition.LEFT; // Record our analysis

            /*
             * Draw a solid rectangle on top of the chosen region.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    REGION1_TOPLEFT_ANCHOR_POINT, // First point which defines the rectangle
                    REGION1_BOTTOMRIGHT_ANCHOR_POINT, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        }
        else if(max == avg2) // Was it from region 2?
        {
            position = TeamElementPosition.CENTER; // Record our analysis

            /*
             * Draw a solid rectangle on top of the chosen region.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    REGION2_TOPLEFT_ANCHOR_POINT, // First point which defines the rectangle
                    REGION2_BOTTOMRIGHT_ANCHOR_POINT, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        }
        else if(max == avg3) // Was it from region 3?
        {
            position = TeamElementPosition.RIGHT; // Record our analysis

            /*
             * Draw a solid rectangle on top of the chosen region.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    REGION3_TOPLEFT_ANCHOR_POINT, // First point which defines the rectangle
                    REGION3_BOTTOMRIGHT_ANCHOR_POINT, // Second point which defines the rectangle,
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        }

        /*
         * Render the 'input' buffer to the viewport. But note this is not
         * simply rendering the raw camera feed, because we called functions
         * to add some annotations to this buffer earlier up.
         */
        return input;
    }

    /*
     * Call this from the OpMode thread to obtain the latest analysis
     */
    public SpikeMarkPosition getAnalysis()
    {
        switch (position) {
            case LEFT:
                return SpikeMarkPosition.UNO;
            case CENTER:
                return SpikeMarkPosition.DOS;
            case RIGHT:
                return SpikeMarkPosition.TRES;
            default:
                return SpikeMarkPosition.UNO;
        }
    }
}
