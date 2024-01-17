/* Copyright (c) 2017 FIRST. All rights reserved.
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
 */

package org.firstinspires.ftc.teamcode.test;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.constants.TeleopServoConstants;
import org.firstinspires.ftc.teamcode.constants.TestServoConstants;
import org.firstinspires.ftc.teamcode.rr.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.lang.Math;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="TestTeleop", group="Robot")
public class KongTeleopTest extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    MecanumDrive drive;
    TrajectoryActionBuilder aB;
    public static boolean isArmMoving = false;
    public static boolean isPokerMoving = false;
    public static boolean isRobotInTrajectory = false;
    // Declare OpMode members.
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
    private Servo PlaneLauncher = null;
    private boolean oldCrossPressed = true;
    private boolean oldTrianglePressed = true;
    private boolean oldCirclePressed = true;
    private boolean oldSquarePressed = true;
    private boolean oldLBumper = true;
    private boolean clawIsClosed = true;
    private int index = 0;
    private double[] LEServoPositions = TestServoConstants.LEServoPositions;
    private double[] REServoPositions = TestServoConstants.REServoPositions;
    private double[] PokerPositions = TestServoConstants.PokerPositions;
    private double[] RWServoPositions = TestServoConstants.RWServoPositions;
    private double[] RingerPositions = TestServoConstants.RingerPositions;

    private final int DELAY_BETWEEN_MOVES = 100;

    @Override
    public void runOpMode() {

        class setIsArmMoving extends TimerTask {
            boolean val;
            public setIsArmMoving(boolean v) {
                this.val = v;
            }
            public void run() {
                isArmMoving = val;
            }
        }

        class setIsPokerMoving extends TimerTask {
            boolean val;
            public setIsPokerMoving(boolean v) {
                this.val = v;
            }
            public void run() {
                isPokerMoving = val;
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
                RightWristServo.setPosition(RWServoPositions[i]);

                telemetry.addData("index", i);
                telemetry.update();
//                sleep(1000);
            }
        }

        class PutRingerToCertainPosition extends TimerTask {
            int i;
            public PutRingerToCertainPosition(int i) {
                this.i = i;
            }
            public void run() {
                Ringer.setPosition(RingerPositions[i]);
            }
        }

        class PutPokerToCertainPosition extends TimerTask {
            int i;
            public PutPokerToCertainPosition(int i) {
                this.i = i;
            }
            public void run() {
                Poker.setPosition(PokerPositions[i]);
            }
        }

        initAprilTag();

        telemetry.addData("Status", "Initialized");
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
        PlaneLauncher = hardwareMap.get(Servo.class, "PL");

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
        LeftSlide.setDirection(DcMotor.Direction.REVERSE);
        RightSlide.setDirection(DcMotor.Direction.FORWARD);
        LeftElbowServo.setDirection(Servo.Direction.FORWARD);
        RightElbowServo.setDirection(Servo.Direction.REVERSE);
        Poker.setDirection(Servo.Direction.FORWARD);
        RightWristServo.setDirection(Servo.Direction.REVERSE);
        Ringer.setDirection(Servo.Direction.FORWARD);
        PlaneLauncher.setDirection(Servo.Direction.FORWARD);

        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        timer.schedule(new PutGrabberToCertainPosition(0), 0);
        timer.schedule(new LowerArmToCertainServoPosition(0), 0);

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(90)));

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        int armIndex = 0;
        int ringerIndex = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            telemetryAprilTag();

            telemetry.addData("", gamepad1.left_stick_y);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

//            if (gamepad1.dpad_left) {
//                drive.pose = new Pose2d(0, 0, Math.toRadians(90));
//                aB = drive.actionBuilder(drive.pose)
//                        .lineToY(24).endTrajectory();
//                Actions.runBlocking(aB.build());
//            }

            // Send calculated power to wheels
            // KYLE CODE
            double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double FLPower = r * Math.cos(robotAngle) + rightX;
            final double FRPower = r * Math.sin(robotAngle) - rightX;
            final double BLPower = r * Math.sin(robotAngle) + rightX;
            final double BRPower = r * Math.cos(robotAngle) - rightX;

            // Send calculated power to wheels
            if (gamepad1.right_bumper || gamepad1.left_bumper) {
                if (gamepad1.right_stick_x == 0 && gamepad1.right_stick_y == 0) {
                    if (gamepad1.right_bumper) {
                        FLMotor.setPower(1);
                        FRMotor.setPower(-1);
                        BLMotor.setPower(-1);
                        BRMotor.setPower(1);
                    } else {
                        FLMotor.setPower(-1);
                        FRMotor.setPower(1);
                        BLMotor.setPower(1);
                        BRMotor.setPower(-1);
                    }
                } else {
                    if (-gamepad1.right_stick_y > 0) {
                        if (gamepad1.right_bumper) {
                            FLMotor.setPower(1);
                            FRMotor.setPower(-1 + Math.abs(gamepad1.right_stick_y));
                            BLMotor.setPower(-1 + Math.abs(gamepad1.right_stick_y));
                            BRMotor.setPower(1);
                        } else {
                            FLMotor.setPower(-1 + Math.abs(gamepad1.right_stick_y));
                            FRMotor.setPower(1);
                            BLMotor.setPower(1);
                            BRMotor.setPower(-1 + Math.abs(gamepad1.right_stick_y));
                        }
                    } else {
                        if (gamepad1.right_bumper) {
                            FLMotor.setPower(1 - Math.abs(gamepad1.right_stick_y));
                            FRMotor.setPower(-1);
                            BLMotor.setPower(-1);
                            BRMotor.setPower(1 - Math.abs(gamepad1.right_stick_y));
                        } else {
                            FLMotor.setPower(-1);
                            FRMotor.setPower(1 - Math.abs(gamepad1.right_stick_y));
                            BLMotor.setPower(1 - Math.abs(gamepad1.right_stick_y));
                            BRMotor.setPower(-1);
                        }
                    }
                }
            } else {
                FLMotor.setPower(FLPower);
                FRMotor.setPower(FRPower);
                BLMotor.setPower(BLPower);
                BRMotor.setPower(BRPower);
            }

            IntakeMotor.setPower(gamepad2.dpad_up ? 1 : gamepad2.dpad_down ? -1 : 0);
            LeftSlide.setPower(gamepad2.left_stick_y);
            RightSlide.setPower(gamepad2.left_stick_y);

            if (gamepad2.right_trigger > 0) {
                PlaneLauncher.setPosition(0.0);
            }
            if (gamepad2.left_trigger > 0) {
                PlaneLauncher.setPosition(0.57);
            }

            boolean circlePressed = gamepad2.circle;
            boolean trianglePressed = gamepad2.triangle;
            if (index == 0) {
                if (circlePressed && !oldCirclePressed && !isArmMoving) {
                    new setIsArmMoving(true).run();
//                    timer.schedule(new LowerArmToCertainServoPosition(0), 0);
                    timer.schedule(new LowerArmToCertainServoPosition(1), 0 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new LowerArmToCertainServoPosition(2), 1 * DELAY_BETWEEN_MOVES);
//                    timer.schedule(new LowerArmToCertainServoPosition(3), 3 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new setIsArmMoving(false), 1 * DELAY_BETWEEN_MOVES);
                    index = 2;
                }
            } else if (index == 2) {
                if (circlePressed && !oldCirclePressed && !isArmMoving) {
                    new setIsArmMoving(true).run();
                    timer.schedule(new LowerArmToCertainServoPosition(1), 0);
                    timer.schedule(new LowerArmToCertainServoPosition(0), 1 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new setIsArmMoving(false), 1 * DELAY_BETWEEN_MOVES);
                    index = 0;
                } else if (trianglePressed && !oldTrianglePressed && !isArmMoving) {
                    new setIsArmMoving(true).run();
                    timer.schedule(new LowerArmToCertainServoPosition(3), 0);
                    timer.schedule(new LowerArmToCertainServoPosition(4), 7 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new setIsArmMoving(false), 7 * DELAY_BETWEEN_MOVES);
                    index = 3;
                }
            } else if (index == 3) {
                if (circlePressed && !oldCirclePressed && !isArmMoving) {
                    new setIsArmMoving(true).run();
//                    timer.schedule(new LowerArmToCertainServoPosition(3), 0);
                    timer.schedule(new PutRingerToCertainPosition(0), 0);
                    timer.schedule(new LowerArmToCertainServoPosition(5), 0 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new LowerArmToCertainServoPosition(6), 1 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new LowerArmToCertainServoPosition(0), 3 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new setIsArmMoving(false), 3 * DELAY_BETWEEN_MOVES);
                    index = 0;
                }
            }

            // ffffffffffffffffffffffffftttttttttttttttttfffffffffttttt
            boolean crossPressed = gamepad2.cross;
            if (crossPressed && !oldCrossPressed && index == 0 && !isPokerMoving) {
                new setIsPokerMoving(true).run();
                timer.schedule(new PutPokerToCertainPosition(1), 0);
                timer.schedule(new PutPokerToCertainPosition(0), 4000);
                timer.schedule(new setIsPokerMoving(false), 8000);
            }

            boolean squarePressed = gamepad2.square;
            if (squarePressed && !oldSquarePressed && index == 3) {
                Ringer.setPosition(RingerPositions[++ringerIndex % RingerPositions.length]);
            }

            boolean LBumper = gamepad2.left_bumper;
            if (LBumper && !oldLBumper && index == 0) {
                new setIsArmMoving(true).run();
                RightWristServo.setPosition(RWServoPositions[0] + 0.05);
                timer.schedule(new TimerTask() {
                    @Override
                    public void run() {
                        RightWristServo.setPosition(RWServoPositions[0] - 0.05);
                    }
                }, 500);
                timer.schedule(new TimerTask() {
                    @Override
                    public void run() {
                        RightWristServo.setPosition(RWServoPositions[0]);
                    }
                }, 1000);
                timer.schedule(new setIsArmMoving(false), 1000);
            }
            oldCrossPressed = crossPressed;
            oldCirclePressed = circlePressed;
            oldSquarePressed = squarePressed;
            oldTrianglePressed = trianglePressed;
            oldLBumper = LBumper;
        }

        visionPortal.close();
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

            // The following default settings are available to un-comment and edit as needed.
            //.setDrawAxes(false)
            //.setDrawCubeProjection(false)
            //.setDrawTagOutline(true)
            //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
            //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
            //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

            // == CAMERA CALIBRATION ==
            // If you do not manually specify calibration parameters, the SDK will attempt
            // to load a predefined calibration for your camera.
                //Focals (pixels) - Fx: 946.868 Fy: 946.868
        //        Optical center - Cx: 397.024 Cy: 293.943
        //        Radial distortion (Brown's Model)
        //        K1: 0.152515 K2: -0.13222 K3: 0.00047578
        //        P1: 0.0208688 P2: 0.0438113
        //        Skew: 0
            .setLensIntrinsics(946.868, 946.868, 397.024, 293.943)
            // ... these parameters are fx, fy, cx, cy.

            .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()


    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }
}
