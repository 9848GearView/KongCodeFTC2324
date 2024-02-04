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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.constants.TeleopServoConstants;

import java.lang.Math;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.locks.Lock;


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
    public static boolean isArmMoving = false;
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
    private Servo clawL = null;
    private Servo clawR = null;
    private CRServo IntakeServo = null;
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
    private double[] LEServoPositions = TeleopServoConstants.LEServoPositions;
    private double[] REServoPositions = TeleopServoConstants.REServoPositions;
    private double[] clawLPositions = TeleopServoConstants.ClawLPositions;
    private double[] WServoPositions = TeleopServoConstants.WServoPositions;
    private double[] clawRPositions = TeleopServoConstants.ClawRPositions;
    private double[] FingerFPositions = TeleopServoConstants.FingerFPositions;
    private double[] FingerBPositions = TeleopServoConstants.FingerBPositions;

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
                clawL.setPosition(clawLPositions[i]);
                clawR.setPosition(clawRPositions[i]);

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

        telemetry.addData("Status", "sInitialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        FLMotor = hardwareMap.get(DcMotor.class, "FL");
        FRMotor = hardwareMap.get(DcMotor.class, "FR");
        BLMotor = hardwareMap.get(DcMotor.class, "BL");
        BRMotor = hardwareMap.get(DcMotor.class, "BR");
        IntakeServo = hardwareMap.get(CRServo.class, "IN");
        LeftSlide = hardwareMap.get(DcMotor.class, "LS");
        RightSlide = hardwareMap.get(DcMotor.class, "RS");
        LeftElbowServo = hardwareMap.get(Servo.class, "LE");
        RightElbowServo = hardwareMap.get(Servo.class, "RE");
        WristServo = hardwareMap.get(Servo.class, "W");
        fingerF = hardwareMap.get(Servo.class, "FF");
        fingerB = hardwareMap.get(Servo.class, "FB");
        clawL = hardwareMap.get(Servo.class, "CL");
        clawR = hardwareMap.get(Servo.class, "CR");
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
        IntakeServo.setDirection(CRServo.Direction.FORWARD);
        LeftSlide.setDirection(DcMotor.Direction.FORWARD);
        RightSlide.setDirection(DcMotor.Direction.REVERSE);
        LeftElbowServo.setDirection(Servo.Direction.REVERSE);
        RightElbowServo.setDirection(Servo.Direction.FORWARD);
        WristServo.setDirection(Servo.Direction.FORWARD);
        fingerF.setDirection(Servo.Direction.FORWARD);
        clawL.setDirection(Servo.Direction.FORWARD);
        clawR.setDirection(Servo.Direction.FORWARD);

        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        timer.schedule(new PutGrabberToCertainPosition(0), 0);
        timer.schedule(new LowerArmToCertainServoPosition(0), 0);
        timer.schedule(new PutClawsToCertainPosition(0), 0);
        timer.schedule(new fLockPixelToggle(0), 0);
        timer.schedule(new bLockPixelToggle(0), 0);
        timer.schedule(new PutBoxToCertainPosition(0), 0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        int armIndex = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double leftPower;
            double rightPower;

            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            leftPower = Range.clip(drive + turn, -1.0, 1.0);
            rightPower = Range.clip(drive - turn, -1.0, 1.0);

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
            if ((gamepad1.right_trigger > 0) || (gamepad1.left_trigger > 0)) {
                new setIsRobotMoving(true).run();
                if (gamepad1.right_stick_x == 0 && gamepad1.right_stick_y == 0) {
                    if (gamepad1.right_trigger > 0) {
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
                        if (gamepad1.right_trigger > 0) {
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
                        if (gamepad1.right_trigger > 0) {
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
                new setIsRobotMoving(true).run();
                FLMotor.setPower(FLPower);
                FRMotor.setPower(FRPower);
                BLMotor.setPower(BLPower);
                BRMotor.setPower(BRPower);
            }

            if (FLPower == 0 && FRPower == 0 && BLPower == 0 && BRPower == 0) {
                new setIsRobotMoving(false).run();
            }

            IntakeServo.setPower(gamepad2.dpad_up ? 1 : gamepad2.dpad_down ? -1 : 0);
            LeftSlide.setPower(gamepad2.left_stick_y);
            RightSlide.setPower(gamepad2.left_stick_y);

            if (gamepad2.left_bumper && runtime.milliseconds() > 90_000) {
                PlaneLauncher.setPosition(0.0);
            }
            if (gamepad2.left_trigger > 0) {
                PlaneLauncher.setPosition(0.57);
            }

            boolean circlePressed = gamepad2.circle;
            boolean trianglePressed = gamepad2.triangle;
            boolean crossPressed = gamepad2.cross;
            boolean squarePressed = gamepad2.square;
            boolean rBumperPressed = gamepad2.right_bumper;


            if (rBumperPressed && !oldRBumperPressed && !isArmMoving && !isRobotMoving) {
                timer.schedule(new PutClawsToCertainPosition(1), 0 * DELAY_BETWEEN_MOVES);
                timer.schedule(new PutClawsToCertainPosition(2), 1 * DELAY_BETWEEN_MOVES);
                timer.schedule(new PutClawsToCertainPosition(3), 2 * DELAY_BETWEEN_MOVES);
            }


            if (index == 0) {
                if (crossPressed && !oldCrossPressed && !isArmMoving) {
                    if (!fingerLocked) {
                        timer.schedule(new fLockPixelToggle(1), 0 * DELAY_BETWEEN_MOVES);
                        timer.schedule(new bLockPixelToggle(1), 0 * DELAY_BETWEEN_MOVES);
                        timer.schedule(new setIsArmMoving(false), 0 * DELAY_BETWEEN_MOVES);
                        fingerLocked = true;

                    } else if (fingerLocked) {
                        timer.schedule(new fLockPixelToggle(0), 0 * DELAY_BETWEEN_MOVES);
                        timer.schedule(new bLockPixelToggle(0), 0 * DELAY_BETWEEN_MOVES);
                        fingerLocked = false;
                    }
                } else if (circlePressed && !oldCirclePressed && !isArmMoving && fingerLocked) {
                    new setIsArmMoving(true).run();
                    timer.schedule(new PutBoxToCertainPosition(1), 0);
                    index = 2;
                    timer.schedule(new setIsArmMoving(false), 0 * DELAY_BETWEEN_MOVES);
                }
            } else if (index == 2) {
                if (circlePressed && !oldCirclePressed && !isArmMoving) {
                    new setIsArmMoving(true).run();
                    timer.schedule(new PutBoxToCertainPosition(0), 0);
                    index = 0;
                    timer.schedule(new setIsArmMoving(false), 0 * DELAY_BETWEEN_MOVES);
                } else if (trianglePressed && !oldTrianglePressed && !isArmMoving) {
                    new setIsArmMoving(true).run();
                    timer.schedule(new LowerArmToCertainServoPosition(1), 0 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new LowerArmToCertainServoPosition(2), 1 * DELAY_BETWEEN_MOVES);
                    index = 3;
                    timer.schedule(new setIsArmMoving(false), 1 * DELAY_BETWEEN_MOVES);
                }
            } else if (index == 3) {
                if (squarePressed && !oldSquarePressed && !isArmMoving && !firstSquarePressed) {
                    timer.schedule(new fLockPixelToggle(0), 0 * DELAY_BETWEEN_MOVES);
                    firstSquarePressed = true;
                    timer.schedule(new setIsArmMoving(false), 0 * DELAY_BETWEEN_MOVES);
                } else if (squarePressed && !oldSquarePressed && !isArmMoving && firstSquarePressed) {
                    timer.schedule(new bLockPixelToggle(0), 0 * DELAY_BETWEEN_MOVES);
                    firstSquarePressed = false;
                    timer.schedule(new setIsArmMoving(false), 0 * DELAY_BETWEEN_MOVES);
                } else if (circlePressed && !oldCirclePressed && !isArmMoving) {
                    new setIsArmMoving(true).run();
                    timer.schedule(new LowerArmToCertainServoPosition(1), 0);
                    timer.schedule(new PutBoxToCertainPosition(0), 1 * DELAY_BETWEEN_MOVES);
                    index = 0;
                    timer.schedule(new setIsArmMoving(false), 0 * DELAY_BETWEEN_MOVES);
                }
            }

                // ffffffffffffffffffffffffftttttttttttttttttfffffffffttttt


                boolean LBumper = gamepad2.left_bumper;


                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.update();
                oldCrossPressed = crossPressed;
                oldCirclePressed = circlePressed;
                oldSquarePressed = squarePressed;
                oldTrianglePressed = trianglePressed;
                oldRBumperPressed = rBumperPressed;
                oldLBumper = LBumper;
            }
        }
    }
