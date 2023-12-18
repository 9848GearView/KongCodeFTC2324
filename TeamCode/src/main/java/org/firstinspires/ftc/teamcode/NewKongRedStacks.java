/*
 * Copyright (c) 2020 OpenFTC Team
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

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.constants.AutoServoConstants;
import org.firstinspires.ftc.teamcode.rr.MecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Timer;
import java.util.TimerTask;


/*
 * This sample demonstrates a basic (but battle-tested and essentially
 * 100% accurate) method of detecting the TeamElement when lined up with
 * the sample regions over the first 3 stones.
 */
@Autonomous
//@Disabled
public class NewKongRedStacks extends LinearOpMode
{
    enum DriveDirection {
        FORWARD,
        LEFT,
        RIGHT,
        BACKWARD
    }

    enum StartingPositionEnum {
        LEFT,
        RIGHT
    }

    enum SlidePackDirection {
        UP,
        DOWN
    }

    enum SpikeMarkPosition {
        UNO,
        DOS,
        TRES
    }
    private ElapsedTime runtime = new ElapsedTime();
    private Timer timer = new Timer();
    private DcMotor FLMotor = null;
    private DcMotor FRMotor = null;
    private DcMotor BLMotor = null;
    private DcMotor BRMotor = null;
    private DcMotor IntakeMotor = null;
    private DcMotor LeftSlide = null;
    private DcMotor RightSlide = null;
    private Servo LeftElbowServo = null;
    private Servo RightElbowServo = null;
    private Servo Poker = null;
    private Servo RightWristServo = null;
    private Servo Ringer = null;
    private ElapsedTime eTime = new ElapsedTime();


    //    static final double     FORWARD_SPEED = 0.5;
//    static final double     TURN_SPEED    = 0.5;
    private int index = 0;
    private double[] LEServoPositions = AutoServoConstants.LEServoPositions;
    private double[] REServoPositions = AutoServoConstants.REServoPositions;
    private double[] PokerPositions = AutoServoConstants.PokerPositions;
    private double[] RWServoPositions = AutoServoConstants.RWServoPositions;
    private double[] RingerPositions = AutoServoConstants.RingerPositions;
    private final int DELAY_BETWEEN_MOVES = 300;
    public class LowerArmToCertainServoPosition extends TimerTask {
        int i;
        public LowerArmToCertainServoPosition(int i) {
            this.i = i;
        }
        public void run() {
            LeftElbowServo.setPosition(LEServoPositions[i]);
            RightElbowServo.setPosition(REServoPositions[i]);
//            LeftWristServo.setPosition(LWServoPositions[i]);
            RightWristServo.setPosition(RWServoPositions[i]);

//                sleep(1000);
//                telemetry.addData("index", i);
//                telemetry.update();
        }
    }

    public class PutRingerToCertainPosition extends TimerTask {
        int i;
        public PutRingerToCertainPosition(int i) {
            this.i = i;
        }
        public void run() {
            Ringer.setPosition(RingerPositions[i]);
        }
    }

    public class PutPokerToCertainPosition extends TimerTask {
        int i;
        public PutPokerToCertainPosition(int i) {
            this.i = i;
        }
        public void run() {
            Poker.setPosition(PokerPositions[i]);
        }
    }

    OpenCvWebcam webcam;
    TeamElementDeterminationPipeline pipeline;
    StartingPositionEnum sideOfFieldToStartOn = StartingPositionEnum.RIGHT;

    @Override
    public void runOpMode()
    {
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCamera1Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */


        telemetry.addData("Status", "sInitialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        FLMotor = hardwareMap.get(DcMotor.class, "FL");
        FRMotor = hardwareMap.get(DcMotor.class, "FR");
        BLMotor = hardwareMap.get(DcMotor.class, "BL");
        BRMotor = hardwareMap.get(DcMotor.class, "BR");
        IntakeMotor = hardwareMap.get(DcMotor.class, "IN");
        LeftSlide = hardwareMap.get(DcMotor.class, "LS");
        RightSlide = hardwareMap.get(DcMotor.class, "RS");
        LeftElbowServo = hardwareMap.get(Servo.class, "LE");
        RightElbowServo = hardwareMap.get(Servo.class, "RE");
        Poker = hardwareMap.get(Servo.class, "P");
        RightWristServo = hardwareMap.get(Servo.class, "RW");
        Ringer = hardwareMap.get(Servo.class, "R");


        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        FLMotor.setDirection(DcMotor.Direction.REVERSE);
        FRMotor.setDirection(DcMotor.Direction.FORWARD);
        BLMotor.setDirection(DcMotor.Direction.REVERSE);
        BRMotor.setDirection(DcMotor.Direction.FORWARD);
        IntakeMotor.setDirection(DcMotor.Direction.FORWARD);
        LeftSlide.setDirection(DcMotor.Direction.FORWARD);
        RightSlide.setDirection(DcMotor.Direction.REVERSE);
        LeftElbowServo.setDirection(Servo.Direction.FORWARD);
        RightElbowServo.setDirection(Servo.Direction.REVERSE);
        Poker.setDirection(Servo.Direction.FORWARD);
        RightWristServo.setDirection(Servo.Direction.REVERSE);
        Ringer.setDirection(Servo.Direction.FORWARD);

        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Wait for the game to start (driver presses PLAY)

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);        pipeline = new TeamElementDeterminationPipeline();
        webcam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
//        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
                telemetry.addData("erroCode", errorCode);
            }
        });
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-36, -63, Math.PI / 2));
//        timer.schedule(new PutGrabberToCertainPosition(0), 3000);

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.update();
            doActions(drive, sideOfFieldToStartOn, pipeline.getAnalysis());

            // Don't burn CPU cycles busy-looping in this sample
            sleep(15000);
            break;
        }
    }

    public static class TeamElementDeterminationPipeline extends OpenCvPipeline
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
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(0,80);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(160,80);
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(560,80);
        static final Point REGION1_BOTTOMRIGHT_ANCHOR_POINT = new Point(160,320);
        static final Point REGION2_BOTTOMRIGHT_ANCHOR_POINT = new Point(480,160);
        static final Point REGION3_BOTTOMRIGHT_ANCHOR_POINT = new Point(640,320);

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

    public class VomitPixelOnGround implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            IntakeMotor.setPower(0.15);
            timer.schedule(new TimerTask() {
                @Override
                public void run() {
                    IntakeMotor.setPower(0);
                }
            }, 1400);
            return false;
        }
    }

    public class LeavePixelOnGround implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            IntakeMotor.setPower(-0.2);
            timer.schedule(new TimerTask() {
                @Override
                public void run() {
                    IntakeMotor.setPower(0);
                }
            }, 1000);
            return false;
        }
    }

    public class PlacePixelOnBackDrop implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            timer.schedule(new PutRingerToCertainPosition(0), 0);
//            timer.schedule(new LowerArmToCertainServoPosition(0), 3 * DELAY_BETWEEN_MOVES);
//            timer.schedule(new LowerArmToCertainServoPosition(1), 4 * DELAY_BETWEEN_MOVES);
            timer.schedule(new LowerArmToCertainServoPosition(2), 0 * DELAY_BETWEEN_MOVES);
            timer.schedule(new LowerArmToCertainServoPosition(3), 3 * DELAY_BETWEEN_MOVES);
            timer.schedule(new PutRingerToCertainPosition(2), 6 * DELAY_BETWEEN_MOVES);
//            timer.schedule(new LowerArmToCertainServoPosition(4), 7 * DELAY_BETWEEN_MOVES);
//            timer.schedule(new LowerArmToCertainServoPosition(5), 8 * DELAY_BETWEEN_MOVES);
//            timer.schedule(new LowerArmToCertainServoPosition(6), 9 * DELAY_BETWEEN_MOVES);
//            timer.schedule(new LowerArmToCertainServoPosition(7), 10 * DELAY_BETWEEN_MOVES);
//            timer.schedule(new LowerArmToCertainServoPosition(8), 11 * DELAY_BETWEEN_MOVES);
            return false;
        }
    }

    public class GrabPixel implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            timer.schedule(new PutRingerToCertainPosition(2), 0);
            timer.schedule(new LowerArmToCertainServoPosition(4),  1 * DELAY_BETWEEN_MOVES);
            timer.schedule(new LowerArmToCertainServoPosition(5), 6 * DELAY_BETWEEN_MOVES);
            timer.schedule(new LowerArmToCertainServoPosition(6), 11 * DELAY_BETWEEN_MOVES);
            timer.schedule(new LowerArmToCertainServoPosition(0),  15 * DELAY_BETWEEN_MOVES);
            timer.schedule(new PutRingerToCertainPosition(0), 15 * DELAY_BETWEEN_MOVES);

            return false;
        }
    }
    private void doActions(MecanumDrive drive, StartingPositionEnum position, SpikeMarkPosition smp) {
//        smp = SpikeMarkPosition.UNO;
        boolean needInvert = (position != StartingPositionEnum.RIGHT);

        TrajectoryActionBuilder actionBuilder = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(-42, -63))
                .turn(0.00001)
                .lineToY(-35);

        if (smp == SpikeMarkPosition.TRES) {
            actionBuilder = actionBuilder
                    .turn(-Math.PI/2)
                    .lineToX(-36)
                    .afterTime(0, new VomitPixelOnGround())
                    .afterTime(1.7, new LeavePixelOnGround())
                    .waitSeconds(2)
                    .strafeTo(new Vector2d(-37, -60))
                    .turn(Math.PI + 0.00001);
        } else if (smp == SpikeMarkPosition.DOS) {
            actionBuilder = actionBuilder
                    .afterTime(0, new VomitPixelOnGround())
                    .afterTime(1.7, new LeavePixelOnGround())
                    .waitSeconds(2)
                    .lineToY(-60)
                    .turn(Math.PI/2);
        } else {
            actionBuilder = actionBuilder
                    .turn(Math.PI / 2)
                    .lineToX(-36)
                    .afterTime(0, new VomitPixelOnGround())
                    .afterTime(1.7, new LeavePixelOnGround())
                    .waitSeconds(2)
                    .strafeTo(new Vector2d(-37, -60))
                    .turnTo(Math.PI);
        }

        double pos = -34;
        double pos2 = -12;
        if (smp == SpikeMarkPosition.UNO) {
            pos = -26;
        }
        if (smp == SpikeMarkPosition.DOS) {
            pos2 = -61;
        }
        if (smp == SpikeMarkPosition.TRES) {
            pos = -44;
            pos2 = -61;
        }
        actionBuilder = actionBuilder
                .lineToX(46.5)
                .strafeToConstantHeading(new Vector2d(44, pos))
                .afterTime(0, new PlacePixelOnBackDrop())
                .afterTime(4, new GrabPixel())
                .waitSeconds(4)
                .strafeToConstantHeading(new Vector2d(46, pos2))
                .turn(0.00001)
                .lineToX(60);

        Actions.runBlocking(actionBuilder.build());
    }

    private DriveDirection getCorrectDirection(DriveDirection direction, boolean needInvert) {
        if (!needInvert)
            return direction;

        DriveDirection invertedDirection = direction;
        switch (direction) {
            case LEFT:
                invertedDirection = DriveDirection.RIGHT;
                break;
            case RIGHT:
                invertedDirection = DriveDirection.LEFT;
                break;
            case FORWARD:
                invertedDirection = DriveDirection.BACKWARD;
                break;
            case BACKWARD:
                invertedDirection = DriveDirection.FORWARD;
                break;
            default:
                break;
        }

        return invertedDirection;
    }
}