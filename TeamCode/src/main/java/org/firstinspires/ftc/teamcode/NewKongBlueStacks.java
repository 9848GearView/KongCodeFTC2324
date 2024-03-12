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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.constants.AutoServoConstants;
import org.firstinspires.ftc.teamcode.MecanumDrive;
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
@Autonomous
//@Disabled
public class NewKongBlueStacks extends LinearOpMode
{
    enum StartingPositionEnum {
        LEFT,
        RIGHT
    }
    public static boolean isArmMoving = false;
    public static boolean slideOverride = false;
    public static boolean isRobotMoving = false;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Timer timer = new Timer();
    private DcMotor FLMotor = null;
    private DcMotor FRMotor = null;
    private DcMotor BLMotor = null;
    private DcMotor BRMotor = null;
    private DcMotor LeftSlide = null;
    private DcMotor RightSlide = null;
    private Servo WristServo = null;
    private Servo LeftElbowServo = null;
    private Servo RightElbowServo = null;
    private Servo fingerF = null;
    private Servo fingerB = null;
    private Servo LeftIntakeServo = null;
    private Servo RightIntakeServo = null;
//    private Servo ClawL = null;
//    private Servo ClawR = null;
    private DcMotor IntakeMotor = null;
    private Servo PlaneLauncher = null;
    private boolean oldCrossPressed = true;
    private boolean oldTrianglePressed = true;
    private boolean oldCirclePressed = true;
    private boolean oldSquarePressed = true;
    private boolean oldRBumperPressed = true;
    private boolean oldLBumper = true;

    private boolean firstSquarePressed = false;

    private boolean fingerLocked = false;
    private int index = 0;
    private double[] LEServoPositions = AutoServoConstants.LEServoPositions;
    private double[] REServoPositions = AutoServoConstants.REServoPositions;
    private double[] ClawLPositions = AutoServoConstants.ClawLPositions;
    private double[] WServoPositions = AutoServoConstants.WServoPositions;
    private double[] ClawRPositions = AutoServoConstants.ClawRPositions;
    private double[] FingerFPositions = AutoServoConstants.FingerFPositions;
    private double[] FingerBPositions = AutoServoConstants.FingerBPositions;
    private double[] LeftIntakePositions = AutoServoConstants.LeftIntakePositions;
    private double[] RightIntakePositions = AutoServoConstants.RightIntakePositions;
    private final int DELAY_BETWEEN_MOVES = 300;

    OpenCvWebcam webcam;
    BlueTeamElementDeterminationPipeline pipeline;
    StartingPositionEnum sideOfFieldToStartOn = StartingPositionEnum.LEFT;

    class setIsArmMoving extends TimerTask {
        boolean val;

        public setIsArmMoving(boolean v) {
            this.val = v;
        }

        public void run() {
            isArmMoving = val;
        }
    }

    class setIsRobotMoving extends TimerTask {
        boolean val;

        public setIsRobotMoving(boolean v) {
            this.val = v;
        }

        public void run() {
            isRobotMoving = val;
        }
    }


    class LowerArmToCertainServoPosition extends TimerTask {
        int i;

        public LowerArmToCertainServoPosition(int i) {
            this.i = i;
        }

        public void run() {
            LeftElbowServo.setPosition(LEServoPositions[i]);
            RightElbowServo.setPosition(REServoPositions[i]);

            telemetry.addData("index", i);
            telemetry.update();

        }
    }

//    class PutClawsToCertainPosition extends TimerTask {
//        int i;
//
//        public PutClawsToCertainPosition(int i) {
//            this.i = i;
//        }
//
//        public void run() {
//            ClawL.setPosition(ClawLPositions[i]);
//            ClawR.setPosition(ClawRPositions[i]);
//
//            telemetry.addData("index", i);
//            telemetry.update();
//
//        }
//    }
    class fLockPixelToggle extends TimerTask {
        int i;

        public fLockPixelToggle(int i) {
            this.i = i;
        }

        public void run() {
            fingerF.setPosition(FingerFPositions[i]);


            telemetry.addData("index", i);
            telemetry.update();

        }
    }

    class bLockPixelToggle extends TimerTask {
        int i;

        public bLockPixelToggle(int i) {
            this.i = i;
        }

        public void run() {
            fingerB.setPosition(FingerBPositions[i]);

            telemetry.addData("index", i);
            telemetry.update();

        }
    }
    class SetBoxToCertainPosition extends TimerTask {
        int i;

        public SetBoxToCertainPosition(int i) {
            this.i = i;
        }

        public void run() {
            WristServo.setPosition(WServoPositions[i]);

            telemetry.addData("index", i);
            telemetry.update();

        }
    }

    class FixCadderMistake extends TimerTask {
        double i;

        public FixCadderMistake(double i) {
            this.i = i;
        }

        public void run() {
            LeftSlide.setPower(i);
            RightSlide.setPower(i);

            telemetry.addData("Power", i);
            telemetry.update();

        }
    }

    class SetSlideOverride extends TimerTask {
        boolean val;

        public SetSlideOverride(boolean v) {
            this.val = v;
        }

        public void run() { slideOverride = val; }
    }

    class SetPowerOfSlides extends TimerTask {
        double p;

        public SetPowerOfSlides(double p) {
            this.p = p;
        }

        public void run() {
            timer.schedule(new TimerTask() {
                @Override
                public void run() {
                    LeftSlide.setPower(p); RightSlide.setPower(p);
                }
            }, 0);
        }
    }

    class PutBoxToCertainPosition extends TimerTask {
        int i;

        public PutBoxToCertainPosition(int i) {
            this.i = i;
        }

        public void run() {
            WristServo.setPosition(WServoPositions[i]);

            telemetry.addData("index", i);
            telemetry.update();

        }
    }

    class PutIntakeToCertainPosition extends TimerTask {
        int i;

        public PutIntakeToCertainPosition(int i) {
            this.i = i;
        }

        public void run() {
            LeftIntakeServo.setPosition(LeftIntakePositions[i]);
            RightIntakeServo.setPosition(RightIntakePositions[i]);

            telemetry.addData("intake index", i);
            telemetry.update();

        }
    }

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
        WristServo = hardwareMap.get(Servo.class, "W");
        fingerF = hardwareMap.get(Servo.class, "FF");
        fingerB = hardwareMap.get(Servo.class, "FB");
        LeftIntakeServo = hardwareMap.get(Servo.class, "LI");
        RightIntakeServo = hardwareMap.get(Servo.class, "RI");
        PlaneLauncher = hardwareMap.get(Servo.class, "PL");

        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        FLMotor.setDirection(DcMotor.Direction.REVERSE);
        FRMotor.setDirection(DcMotor.Direction.FORWARD);
        BLMotor.setDirection(DcMotor.Direction.REVERSE);
        BRMotor.setDirection(DcMotor.Direction.REVERSE);
        IntakeMotor.setDirection(CRServo.Direction.FORWARD);
        LeftSlide.setDirection(DcMotor.Direction.REVERSE);
        RightSlide.setDirection(DcMotor.Direction.FORWARD);
        LeftElbowServo.setDirection(Servo.Direction.REVERSE);
        RightElbowServo.setDirection(Servo.Direction.FORWARD);
        WristServo.setDirection(Servo.Direction.FORWARD);
        fingerF.setDirection(Servo.Direction.FORWARD);
        fingerB.setDirection(Servo.Direction.FORWARD);
        LeftIntakeServo.setDirection(Servo.Direction.FORWARD);
        RightIntakeServo.setDirection(Servo.Direction.REVERSE);
        PlaneLauncher.setDirection(Servo.Direction.REVERSE);

        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        new LowerArmToCertainServoPosition(0).run();
//        new PutClawsToCertainPosition(0).run();
        new fLockPixelToggle(1).run();
        new bLockPixelToggle(1).run();
        new PutBoxToCertainPosition(1).run();
        new PutIntakeToCertainPosition(2).run();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new BlueTeamElementDeterminationPipeline();
        webcam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
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
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-36, 64, -Math.PI / 2));

        waitForStart();

        while (opModeIsActive())
        {
//            telemetry.addData("Analysis", pipeline.getAnalysis());
//            telemetry.update();
            doActions(drive, sideOfFieldToStartOn, pipeline.getAnalysis());

            // Don't burn CPU cycles busy-looping in this sample
            sleep(15000);
            break;
        }
    }

    public class VomitPixelOnGround implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {timer.schedule(new fLockPixelToggle(0), 0 * DELAY_BETWEEN_MOVES);
            timer.schedule(new fLockPixelToggle(0), 0 * DELAY_BETWEEN_MOVES);
//            timer.schedule(new SetBoxToCertainPosition(0), 1000);
            return false;
        }
    }

    public class VomitPixelOnBackdrop implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {timer.schedule(new fLockPixelToggle(0), 0 * DELAY_BETWEEN_MOVES);
            timer.schedule(new bLockPixelToggle(0), 0 * DELAY_BETWEEN_MOVES);
            timer.schedule(new fLockPixelToggle(0), 0 * DELAY_BETWEEN_MOVES);
            return false;
        }
    }

    public class PlacePixelOnBackDrop implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            timer.schedule(new LowerArmToCertainServoPosition(1), 0 * DELAY_BETWEEN_MOVES);
            timer.schedule(new LowerArmToCertainServoPosition(2), 1 * DELAY_BETWEEN_MOVES); //no 3 index
            timer.schedule(new SetBoxToCertainPosition(1), 1 * DELAY_BETWEEN_MOVES);
            return false;
        }
    }

    public class PlacePixelOnGround implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            timer.schedule(new LowerArmToCertainServoPosition(1), 0 * DELAY_BETWEEN_MOVES);
            timer.schedule(new LowerArmToCertainServoPosition(3), 1 * DELAY_BETWEEN_MOVES);
            timer.schedule(new SetBoxToCertainPosition(2), 0);
            return false;
        }
    }

    public class GrabPixel implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            timer.schedule(new LowerArmToCertainServoPosition(0), 0 * DELAY_BETWEEN_MOVES);
            timer.schedule(new SetBoxToCertainPosition(0), 0 * DELAY_BETWEEN_MOVES);
            return false;
        }
    }

    public class RaiseIntake implements Action {
        int i;
        public RaiseIntake(int i) {
            this.i = i;
        }
        public RaiseIntake() {
            this.i = 2;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            new PutIntakeToCertainPosition(i).run();
            return false;
        }
    }

    public class RaiseArm implements Action {
        double p;
        int t;
        public RaiseArm(double p, int t) {
            this.p = p;
            this.t = t;
        }
        public RaiseArm() {
            this.p = 0.5;
            this.t = 500;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            timer.schedule(new SetPowerOfSlides(p), 0);
            timer.schedule(new SetPowerOfSlides(0), t);
            return false;
        }
    }

    public class LowerArm implements Action {
        double p;
        int t;
        public LowerArm(double p, int t) {
            this.p = -p;
            this.t = t;
        }
        public LowerArm() {
            this.p = -0.5;
            this.t = 500;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            timer.schedule(new SetPowerOfSlides(p), 0);
            timer.schedule(new SetPowerOfSlides(0), t);
            return false;
        }
    }

    private void doActions(MecanumDrive drive, StartingPositionEnum position, SpikeMarkPosition smp) {
//        sleep(5000);
//        smp = SpikeMarkPosition.UNO;
        boolean needInvert = (position != StartingPositionEnum.RIGHT);
        double multiplier = 1;
        if (needInvert) {
            multiplier = -1;
        }

        TrajectoryActionBuilder actionBuilder = drive.actionBuilder(drive.pose)
                .afterTime(0, new RaiseIntake(0))
                .turn(multiplier * 0.00001)
                .afterTime(0, new RaiseArm(0.5, 1000));

        if (smp == SpikeMarkPosition.UNO) {
            actionBuilder = actionBuilder
                    .strafeTo(new Vector2d(-43, multiplier * -36))
                    .afterTime(.5, new PlacePixelOnGround())
                    .turnTo(Math.PI + 0.00001)
                    .lineToX(-38)
                    .afterTime(0, new LowerArm(0.5, 400))
                    .afterTime(1.5, new VomitPixelOnGround())
                    .afterTime(2, new RaiseArm(0.5, 400))
                    .waitSeconds(3)
                    .lineToX(-45)
                    .strafeTo(new Vector2d(-46, multiplier * -10.5))
                    .turnTo(Math.PI);
        } else if (smp == SpikeMarkPosition.DOS) {
            actionBuilder = actionBuilder
                    .afterTime(0, new RaiseIntake(0))
                    .strafeTo(new Vector2d(-48, multiplier * -12))
                    .strafeTo(new Vector2d(-36, multiplier * -12))
                    .afterTime(.5, new PlacePixelOnGround())
                    .afterTime(0, new LowerArm(0.5, 400))
                    .afterTime(2, new VomitPixelOnGround())
                    .afterTime(2, new RaiseIntake(2))
                    .afterTime(2.6, new RaiseArm(0.5, 400))
                    .waitSeconds(3)
                    .turnTo(Math.PI);
        } else {
            actionBuilder = actionBuilder
                    .afterTime(0, new RaiseIntake(0))
                    .strafeTo(new Vector2d(-46, multiplier * -12))
                    .afterTime(.5, new PlacePixelOnGround())
                    .afterTime(0, new LowerArm(0.5, 400))
                    .afterTime(2, new VomitPixelOnGround())
                    .afterTime(2, new RaiseIntake(2))
                    .afterTime(2.6, new RaiseArm(0.5, 400))
                    .waitSeconds(3)
                    .turnTo(Math.PI);
        }

        actionBuilder = actionBuilder
                .afterTime(0, new LowerArm(0.5, 700))
                .afterTime(0, new PlacePixelOnBackDrop())
                .lineToX(24)
                .turnTo(Math.PI);

        double pos = -33.75;
        double pos2 = -8;
        if (smp == SpikeMarkPosition.UNO) {
            pos = -39.5;
            pos2 = -58;
        }
        if (smp == SpikeMarkPosition.DOS) {
            pos2 = -8;
        }
        if (smp == SpikeMarkPosition.TRES) {
            pos = -27;
            pos2 = -10;
        }
        actionBuilder = actionBuilder
                .afterTime(0, new RaiseArm(0.5, 800))
                .strafeToConstantHeading(new Vector2d(52, multiplier * pos))
                .afterTime(2.5, new VomitPixelOnBackdrop())
                .afterTime(3.2, new RaiseArm())
                .afterTime(3.3, new GrabPixel())
                .afterTime(2.5, new RaiseIntake(0))
                .waitSeconds(3)
                .strafeToConstantHeading(new Vector2d(42, multiplier * pos2))
                .turn(multiplier * 0.00001)
                .lineToX(60)
                .afterTime(0, new LowerArm(0.5, 700));

        // For testing placing on backdrop
//        actionBuilder = drive.actionBuilder(drive.pose)
//                        .afterTime(0, new RaiseArm())
//                        .afterTime(.5, new PlacePixelOnGround())
//                        .afterTime(1.7, new VomitPixelOnGround())
//                        .afterTime(4, new PlacePixelOnBackDrop())
//                        .afterTime(5, new LowerArm());

        Actions.runBlocking(actionBuilder.build());
    }
}