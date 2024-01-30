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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RedTeamElementDeterminationPipeline;
import org.firstinspires.ftc.teamcode.SpikeMarkPosition;
import org.firstinspires.ftc.teamcode.constants.AutoServoConstants;
import org.firstinspires.ftc.teamcode.rr.MecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Timer;
import java.util.TimerTask;


/*
 * This sample demonstrates a basic (but battle-tested and essentially
 * 100% accurate) method of detecting the TeamElement when lined up with
 * the sample regions over the first 3 stones.
 */
@Autonomous(name = "KongRedBackdrop")
//@Disabled
public class NewKongRedBackdrop extends LinearOpMode
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
    RedTeamElementDeterminationPipeline pipeline;
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
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new RedTeamElementDeterminationPipeline();
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

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(12, -63, Math.PI / 2));
//        timer.schedule(new PutGrabberToCertainPosition(0), 3000);

        waitForStart();

        while (opModeIsActive())
        {
//            sleep(1000);
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.update();
            doActions(drive, sideOfFieldToStartOn, pipeline.getAnalysis());

            // Don't burn CPU cycles busy-looping in this sample
            sleep(15000);
            break;
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
            timer.schedule(new LowerArmToCertainServoPosition(0),  20 * DELAY_BETWEEN_MOVES);
            timer.schedule(new PutRingerToCertainPosition(0), 20 * DELAY_BETWEEN_MOVES);
            return false;
        }
    }

    public class RaiseArm implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            LeftSlide.setPower(0.43);
            RightSlide.setPower(0.43);
            timer.schedule(new TimerTask() {
                @Override
                public void run() {
                    LeftSlide.setPower(0); RightSlide.setPower(0);
                }
            }, 400);
            return false;
        }
    }

    private void doActions(MecanumDrive drive, StartingPositionEnum position, SpikeMarkPosition smp) {
//        smp = SpikeMarkPosition.UNO;
        boolean needInvert = (position != StartingPositionEnum.RIGHT);

        TrajectoryActionBuilder actionBuilder = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(17, -63))
                .turn(0.00001)
                .lineToY(-36);

        if (smp == SpikeMarkPosition.UNO) {
            actionBuilder = actionBuilder
                    .turn(Math.PI/2)
                    .lineToX(11)
                    .afterTime(0, new VomitPixelOnGround())
                    .afterTime(1.7, new LeavePixelOnGround())
                    .waitSeconds(2);
        } else if (smp == SpikeMarkPosition.DOS) {
            actionBuilder = actionBuilder
                    .strafeTo(new Vector2d(15, -36))
                    .afterTime(0, new VomitPixelOnGround())
                    .afterTime(1.7, new LeavePixelOnGround())
                    .waitSeconds(2)
                    .lineToY(-48)
                    .turn(Math.PI/2);
        } else {
            actionBuilder = actionBuilder
                    .turn(Math.PI / 2)
                    .lineToX(34)
                    .afterTime(0, new VomitPixelOnGround())
                    .afterTime(1.7, new LeavePixelOnGround())
                    .waitSeconds(2);
        }

        double pos = -36;
        double pos2 = -12;
        if (smp == SpikeMarkPosition.UNO) {
            pos = -28;
            pos2 = -61;
        }
        if (smp == SpikeMarkPosition.DOS) {
            pos2 = -61;
        }
        if (smp == SpikeMarkPosition.TRES) {
            pos = -44;
        }
        actionBuilder = actionBuilder
                .lineToX(47)
                .strafeToConstantHeading(new Vector2d(44, pos))
                .afterTime(0, new RaiseArm())
                .afterTime(1, new PlacePixelOnBackDrop())
                .afterTime(5, new GrabPixel())
                .waitSeconds(5)
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