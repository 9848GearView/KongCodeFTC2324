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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.constants.TeleopServoConstants;
import org.firstinspires.ftc.teamcode.constants.TestServoConstants;

import java.lang.Math;
import java.sql.Time;
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

@TeleOp(name="NewKongTeleop", group="Robot")
public class NewKongTeleop extends LinearOpMode {
    public static boolean manualIntakeControl = false;
    public static int intakePos = 2;
    public static boolean isArmMoving = false;
    public static boolean slideOverride = false;
    public static boolean isRobotMoving = false;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Timer timer = new Timer();
    private ColorSensor frontColorSensor;
    private ColorSensor backColorSensor;
    private AnalogInput frontAnalogInput;
    private AnalogInput backAnalogInput;
    private boolean isIntakeUp = false;
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
    private DcMotor IntakeMotor = null;
    private Servo PlaneLauncher = null;
    private boolean oldCrossPressed = true;
    private boolean oldTrianglePressed = true;
    private boolean oldCirclePressed = true;
    private boolean oldSquarePressed = true;
    private boolean oldRBumperPressed = true;
    private double oldRTriggerPressed = 0;
    private boolean oldLBumper = true;
    private boolean oldStartPressed = true;
    private boolean oldBackPressed = true;
    private boolean firstSquarePressed = false;
    private boolean oldDpadLeft = true;
    private boolean oldDpadRight = true;
    private boolean fingersLocked = false;
    private boolean backFingerLocked = false;
    private boolean fingerMovementFinished = true;
    private boolean intakeMoving = false;
    private int index = 0; // 0 is ready to intake, 2 is intermediate, 3 is ready to place
    private double[] LEServoPositions = TeleopServoConstants.LEServoPositions;
    private double[] REServoPositions = TeleopServoConstants.REServoPositions;
    private double[] ClawLPositions = TeleopServoConstants.ClawLPositions;
    private double[] WServoPositions = TeleopServoConstants.WServoPositions;
    private double[] ClawRPositions = TeleopServoConstants.ClawRPositions;
    private double[] FingerFPositions = TeleopServoConstants.FingerFPositions;
    private double[] FingerBPositions = TeleopServoConstants.FingerBPositions;
    private double[] LeftIntakePositions = TeleopServoConstants.LeftIntakePositions;
    private double[] RightIntakePositions = TeleopServoConstants.RightIntakePositions;
    private double[] LauncherPosition = TeleopServoConstants.LauncherPosition;

    private DigitalChannel LDLEDG;
    private DigitalChannel LULEDG;
    private DigitalChannel RDLEDG;
    private DigitalChannel RULEDG;
    private DigitalChannel LDLEDR;
    private DigitalChannel LULEDR;
    private DigitalChannel RDLEDR;
    private DigitalChannel RULEDR;

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

        class PutClawsToCertainPosition extends TimerTask {
            int i;

            public PutClawsToCertainPosition(int i) {
                this.i = i;
            }

            public void run() {
                LeftIntakeServo.setPosition(ClawLPositions[i]);
                RightIntakeServo.setPosition(ClawRPositions[i]);

                telemetry.addData("index", i);
                telemetry.update();

            }
        }
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

        class SetSlideOverride extends TimerTask {
            boolean val;

            public SetSlideOverride(boolean v) {
                this.val = v;
            }

            public void run() { slideOverride = val; }
        }

        class FixCaddersMistake extends TimerTask {
            double p;

            public FixCaddersMistake(double p) {
                this.p = p;
            }

            public void run() {
                LeftSlide.setPower(p);
                RightSlide.setPower(p);

                new SetSlideOverride(p != 0).run();

                telemetry.addData("Power", p);
                telemetry.update();

            }
        }

        class setFingerMovementFinished extends TimerTask {
            boolean val;

            public setFingerMovementFinished(boolean v) {
                this.val = v;
            }

            public void run() { fingerMovementFinished = val; }
        }

        class setLauncherPosition extends TimerTask {
            int pos;

            public setLauncherPosition(int pos) { this.pos = pos; }

            public void run() {
                PlaneLauncher.setPosition(LauncherPosition[pos]);
            }
        }

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
        backAnalogInput = hardwareMap.get(AnalogInput.class, "B");
        frontAnalogInput = hardwareMap.get(AnalogInput.class, "F");

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
        IntakeMotor.setDirection(DcMotor.Direction.FORWARD);
        LeftSlide.setDirection(DcMotor.Direction.REVERSE);
        RightSlide.setDirection(DcMotor.Direction.FORWARD);
        LeftElbowServo.setDirection(Servo.Direction.REVERSE);
        RightElbowServo.setDirection(Servo.Direction.FORWARD);
        WristServo.setDirection(Servo.Direction.FORWARD);
        fingerF.setDirection(Servo.Direction.FORWARD);
        LeftIntakeServo.setDirection(Servo.Direction.FORWARD);
        RightIntakeServo.setDirection(Servo.Direction.REVERSE);
        PlaneLauncher.setDirection(Servo.Direction.REVERSE);

        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LDLEDG = hardwareMap.get(DigitalChannel.class, "LDLEDG");
        LULEDG = hardwareMap.get(DigitalChannel.class, "LULEDG");
        RDLEDG = hardwareMap.get(DigitalChannel.class, "RDLEDG");
        RULEDG = hardwareMap.get(DigitalChannel.class, "RULEDG");
        LDLEDG.setMode(DigitalChannel.Mode.OUTPUT);
        LULEDG.setMode(DigitalChannel.Mode.OUTPUT);
        RDLEDG.setMode(DigitalChannel.Mode.OUTPUT);
        RULEDG.setMode(DigitalChannel.Mode.OUTPUT);

        LDLEDR = hardwareMap.get(DigitalChannel.class, "LDLEDR");
        LULEDR = hardwareMap.get(DigitalChannel.class, "LULEDR");
        RDLEDR = hardwareMap.get(DigitalChannel.class, "RDLEDR");
        RULEDR = hardwareMap.get(DigitalChannel.class, "RULEDR");
        LDLEDR.setMode(DigitalChannel.Mode.OUTPUT);
        LULEDR.setMode(DigitalChannel.Mode.OUTPUT);
        RDLEDR.setMode(DigitalChannel.Mode.OUTPUT);
        RULEDR.setMode(DigitalChannel.Mode.OUTPUT);

        backColorSensor = hardwareMap.get(ColorSensor.class, "CSF");
        frontColorSensor = hardwareMap.get(ColorSensor.class, "CSB");

        new LowerArmToCertainServoPosition(0).run();
        new PutClawsToCertainPosition(0).run();
        new fLockPixelToggle(0).run();
        new bLockPixelToggle(0).run();
        new PutBoxToCertainPosition(0).run();
        PlaneLauncher.setPosition(0.79);

        // Wait for the game to start (driver presses PLAY)

        waitForStart();
        runtime.reset();

        int armIndex = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Send calculated power to wheels
            // KYLE CODE
            double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            double FLPower = r * Math.cos(robotAngle) + rightX;
            double FRPower = r * Math.sin(robotAngle) - rightX;
            double BLPower = r * Math.sin(robotAngle) + rightX;
            double BRPower = r * Math.cos(robotAngle) - rightX;

//            double ratio = 1.0 / Math.max(Math.max(Math.abs(BLPower), Math.abs(BRPower)),
//                    Math.max(Math.abs(FLPower), Math.abs(FRPower)));

//            FLPower *= ratio;
//            FRPower *= ratio;
//            BLPower *= ratio;
//            BRPower *= ratio;

            // Send calculated power to wheels
            if ((gamepad1.right_trigger > 0) || (gamepad1.left_trigger > 0)) {
                new setIsRobotMoving(true).run();
                if (gamepad1.right_stick_x == 0 && gamepad1.right_stick_y == 0) {
                    if (gamepad1.right_trigger > 0) {
                        FLMotor.setPower(gamepad1.right_trigger);
                        FRMotor.setPower(-gamepad1.right_trigger);
                        BLMotor.setPower(-gamepad1.right_trigger);
                        BRMotor.setPower(gamepad1.right_trigger);
                    } else {
                        FLMotor.setPower(-gamepad1.left_trigger);
                        FRMotor.setPower(gamepad1.left_trigger);
                        BLMotor.setPower(gamepad1.left_trigger);
                        BRMotor.setPower(-gamepad1.left_trigger);
                    }
                }
//                else {
//                    if (-gamepad1.right_stick_y > 0) {
//                        if (gamepad1.right_trigger > 0) {
//                            FLMotor.setPower(1);
//                            FRMotor.setPower(-1 + Math.abs(gamepad1.right_stick_y));
//                            BLMotor.setPower(-1 + Math.abs(gamepad1.right_stick_y));
//                            BRMotor.setPower(1);
//                        } else {
//                            FLMotor.setPower(-1 + Math.abs(gamepad1.right_stick_y));
//                            FRMotor.setPower(1);
//                            BLMotor.setPower(1);
//                            BRMotor.setPower(-1 + Math.abs(gamepad1.right_stick_y));
//                        }
//                    } else {
//                        if (gamepad1.right_trigger > 0) {
//                            FLMotor.setPower(1 - Math.abs(gamepad1.right_stick_y));
//                            FRMotor.setPower(-1);
//                            BLMotor.setPower(-1);
//                            BRMotor.setPower(1 - Math.abs(gamepad1.right_stick_y));
//                        } else {
//                            FLMotor.setPower(-1);
//                            FRMotor.setPower(1 - Math.abs(gamepad1.right_stick_y));
//                            BLMotor.setPower(1 - Math.abs(gamepad1.right_stick_y));
//                            BRMotor.setPower(-1);
//                        }
//                    }
//                }

            } else {
                new setIsRobotMoving(true).run();
                FLMotor.setPower(FLPower);
                FRMotor.setPower(FRPower);
                BLMotor.setPower(BLPower);
                BRMotor.setPower(BRPower);
            }

            if (FLPower == 0 && FRPower == 0 && BLPower == 0 && BRPower == 0) {
                new setIsRobotMoving(false).run();
            }

            if (gamepad1.x) {  //alt strafe for when placing (Dion's request)
                FLMotor.setPower(.6);
                FRMotor.setPower(-.6);
                BLMotor.setPower(-.6);
                BRMotor.setPower(.6);
            } else if (gamepad1.b) {
                FLMotor.setPower(-.6);
                FRMotor.setPower(.6);
                BLMotor.setPower(.6);
                BRMotor.setPower(-.6);
            }

            if (intakePos == 0 && index == 0) { //only if index == 0 ??
                IntakeMotor.setPower(gamepad2.dpad_up ? -0.75 : gamepad2.dpad_down ? 0.75 : 0);
            } else {
                IntakeMotor.setPower(gamepad2.dpad_up ? -0.75 : gamepad2.dpad_down ? 0.75 : 0);
            }

            boolean DpadLeft = gamepad2.dpad_left;
            boolean DpadRight = gamepad2.dpad_right;
            if (DpadLeft && !oldDpadLeft) {
                new PutIntakeToCertainPosition(0).run();
                intakePos = 0;
            } else if (DpadRight && !oldDpadRight) {
                if (intakePos == 0 || intakePos == 2) {
                    new PutIntakeToCertainPosition(1).run();
                    intakePos = 1;
                } else {
                    new PutIntakeToCertainPosition(2).run();
                    intakePos = 2;
                }
            }

            if (!slideOverride) {
                LeftSlide.setPower(-gamepad2.left_stick_y);
                RightSlide.setPower(-gamepad2.left_stick_y);
            }

            if ((gamepad2.left_bumper || gamepad2.left_trigger > 0) && runtime.seconds() > 90) {
                timer.schedule(new setLauncherPosition(0), 0); //position 0.3, release drone
                timer.schedule(new setLauncherPosition(1), 1000); //position 0.79, reset drone launcher
            }

//            if (gamepad2.left_trigger > 0 && runtime.seconds() > 90) {
//                PlaneLauncher.setPosition(0.79);
//            }
//
//            if (gamepad2.left_bumper && runtime.seconds() > 90) {
//                PlaneLauncher.setPosition(0.3);
//            }

//            if (runtime.milliseconds() > 118_000 && !isIntakeUp) { // intake up time decided on by team
//                new PutIntakeToCertainPosition(2).run();
//                isIntakeUp = true;
//            }

            boolean circlePressed = gamepad2.circle;
            boolean trianglePressed = gamepad2.triangle;
            boolean crossPressed = gamepad2.cross;
            boolean squarePressed = gamepad2.square;
            boolean rBumperPressed = gamepad2.right_bumper;
            double rTriggerPressed = gamepad2.right_trigger;
            boolean startPressed = gamepad2.start;
            boolean backPressed = gamepad2.back;

            if (startPressed && !oldStartPressed) {
                manualIntakeControl = !manualIntakeControl;
            }

            if (index == 3) {
                if (rTriggerPressed > 0 && oldRTriggerPressed < 0.01) {
                    WServoPositions[2] = Math.max(0, WServoPositions[2] - 0.03);
                    new PutBoxToCertainPosition(2).run();
                }

                if (rBumperPressed && oldRBumperPressed) {
                    WServoPositions[2] = Math.min(1, WServoPositions[2] + 0.03);
                    new PutBoxToCertainPosition(2).run();
                }
            }
            if (backPressed && !oldBackPressed /*&& fingersLocked*/) {  //for when it doesn't lock correctly (Mrs. B Request)
                timer.schedule(new fLockPixelToggle(2), 0 * DELAY_BETWEEN_MOVES);
                timer.schedule(new bLockPixelToggle(2), 0 * DELAY_BETWEEN_MOVES);
                timer.schedule(new setFingerMovementFinished(false), 0 * DELAY_BETWEEN_MOVES);
                fingersLocked = true;
            }
            if (index == 0) { //bucket down
                if (crossPressed && !oldCrossPressed && !isArmMoving) { //grab
                    if (!fingersLocked) {
                        timer.schedule(new fLockPixelToggle(1), 0 * DELAY_BETWEEN_MOVES);
                        timer.schedule(new bLockPixelToggle(1), 0 * DELAY_BETWEEN_MOVES);
                        backFingerLocked = true;
                        fingersLocked = true;
                    } else { //release
                        timer.schedule(new fLockPixelToggle(0), 0 * DELAY_BETWEEN_MOVES);
                        timer.schedule(new bLockPixelToggle(0), 0 * DELAY_BETWEEN_MOVES);
                        backFingerLocked = false;
                        fingersLocked = false;
                    }
                }
                if (circlePressed && !oldCirclePressed && !isArmMoving) { //bucket up
                    new setIsArmMoving(true).run();
                    timer.schedule(new fLockPixelToggle(1), 0 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new bLockPixelToggle(1), 0 * DELAY_BETWEEN_MOVES);
                    backFingerLocked = true;
                    fingersLocked = true;
                    timer.schedule(new setFingerMovementFinished(false), 0 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new FixCaddersMistake(1), 0 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new FixCaddersMistake(0), 4 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new PutBoxToCertainPosition(1), 4 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new setFingerMovementFinished(true), 2 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new setIsArmMoving(false), 0 * DELAY_BETWEEN_MOVES);
                    index = 2;
                }
            } else if (index == 2) { //bucket up
                if (circlePressed && !oldCirclePressed && !isArmMoving) { //go back down
                    new setIsArmMoving(true).run();
//                    timer.schedule(new FixCaddersMistake(1), 0 * DELAY_BETWEEN_MOVES);
//                    timer.schedule(new FixCaddersMistake(0), 4 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new PutBoxToCertainPosition(0), 0 * DELAY_BETWEEN_MOVES);
//                    timer.schedule(new FixCaddersMistake(-1), 2 * DELAY_BETWEEN_MOVES);
//                    timer.schedule(new FixCaddersMistake(0), 6 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new setIsArmMoving(false), 0 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new fLockPixelToggle(0), 0 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new bLockPixelToggle(0), 0 * DELAY_BETWEEN_MOVES);
                    backFingerLocked = false;
                    fingersLocked = false;
                    index = 0;
                } else if (trianglePressed && !oldTrianglePressed && !isArmMoving) { //flip arm
                    new setIsArmMoving(true).run();
                    timer.schedule(new LowerArmToCertainServoPosition(1), 0 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new LowerArmToCertainServoPosition(2), 1 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new PutBoxToCertainPosition(2), 1 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new setIsArmMoving(false), 1 * DELAY_BETWEEN_MOVES);
                    index = 3;
                }
            } else if (index == 3) { //ready to place
                if (squarePressed && !oldSquarePressed && !isArmMoving) { //release 1
                    if (!firstSquarePressed) {
                        timer.schedule(new fLockPixelToggle(0), 0 * DELAY_BETWEEN_MOVES);
                        timer.schedule(new setIsArmMoving(false), 0 * DELAY_BETWEEN_MOVES);
                        firstSquarePressed = true;
                    } else { //release back
                        timer.schedule(new bLockPixelToggle(0), 0 * DELAY_BETWEEN_MOVES);
                        timer.schedule(new setIsArmMoving(false), 0 * DELAY_BETWEEN_MOVES);
                        firstSquarePressed = false;
                        backFingerLocked = false;
                        fingersLocked = false;
                    }
                } else if (circlePressed && !oldCirclePressed && !isArmMoving) { //back to stage 1
                    new setIsArmMoving(true).run();
//                    timer.schedule(new FixCaddersMistake(1), 0 * DELAY_BETWEEN_MOVES);
//                    timer.schedule(new FixCaddersMistake(0), 4 * DELAY_BETWEEN_MOVES);
//                    timer.schedule(new FixCaddersMistake(-1), 4 * DELAY_BETWEEN_MOVES);
//                    timer.schedule(new FixCaddersMistake(0), 8 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new PutBoxToCertainPosition(0), 2 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new LowerArmToCertainServoPosition(1), 2 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new LowerArmToCertainServoPosition(3), 5 * DELAY_BETWEEN_MOVES); //prevent hitting spool
                    timer.schedule(new LowerArmToCertainServoPosition(0), 10 * DELAY_BETWEEN_MOVES); //increase delay as needed
                    timer.schedule(new setIsArmMoving(false), 3 * DELAY_BETWEEN_MOVES);
                    firstSquarePressed = false;
//                    timer.schedule(new fLockPixelToggle(0), 0 * DELAY_BETWEEN_MOVES);
//                    timer.schedule(new bLockPixelToggle(0), 0 * DELAY_BETWEEN_MOVES);
//                    backFingerLocked = false;
//                    fingersLocked = false;
                    index = 0;
                }
            }

            // Lights
            if (Math.abs(fingerF.getPosition() - FingerFPositions[1]) < 0.01) {
                LULEDR.setState(true);
                RULEDR.setState(true);
                LULEDG.setState(false);
                RULEDG.setState(false);
            } else {
                LULEDR.setState(false);
                RULEDR.setState(false);
                LULEDG.setState(true);
                RULEDG.setState(true);
            }

            if (Math.abs(fingerB.getPosition() - FingerBPositions[1]) < 0.01) {
                LDLEDR.setState(true);
                RDLEDR.setState(true);
                LDLEDG.setState(false);
                RDLEDG.setState(false);
            } else {
                LDLEDR.setState(false);
                RDLEDR.setState(false);
                LDLEDG.setState(true);
                RDLEDG.setState(true);
            }

            // ffffffffffffffffffffffffftttttttttttttttttfffffffffttttt
            boolean LBumper = gamepad2.left_bumper;
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("RED", backColorSensor.red());
            telemetry.addData("GREEN", backColorSensor.green());
            telemetry.addData("BLUE", backColorSensor.blue());
            telemetry.addData("RED", frontColorSensor.red());
            telemetry.addData("GREEN", frontColorSensor.green());
            telemetry.addData("BLUE", frontColorSensor.blue());
            telemetry.addData("fVoltage", frontAnalogInput.getVoltage());
            telemetry.addData("bVoltage", backAnalogInput.getVoltage());
            telemetry.addData("dpad_left", DpadLeft);
            telemetry.addData("dpad_right", DpadRight);
            telemetry.addData("pos", intakePos);
            telemetry.addData("manual intake control", manualIntakeControl);
            telemetry.addData("fposition", fingerF.getPosition());
            telemetry.addData("bposition", fingerB.getPosition());
            telemetry.addData("wservobackpos", WServoPositions[2]);
            telemetry.update();
            oldCrossPressed = crossPressed;
            oldCirclePressed = circlePressed;
            oldSquarePressed = squarePressed;
            oldTrianglePressed = trianglePressed;
            oldRBumperPressed = rBumperPressed;
            oldRTriggerPressed = rTriggerPressed;
            oldLBumper = LBumper;
            oldStartPressed = startPressed;
            oldDpadLeft = DpadLeft;
            oldDpadRight = DpadRight;
            oldBackPressed = backPressed;
        }
    }
}
