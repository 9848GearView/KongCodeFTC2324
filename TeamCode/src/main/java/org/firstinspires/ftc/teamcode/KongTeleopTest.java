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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


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

@TeleOp(name="KongTeleopTest", group="Robot")
public class KongTeleopTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FLMotor = null;
    private DcMotor FRMotor = null;
    private DcMotor BLMotor = null;
    private DcMotor BRMotor = null;
    private DcMotor IntakeMotor = null;
    private DcMotor LeftSlide = null;
    private DcMotor RightSlide = null;
    private Servo LeftElbowServo = null;
    private Servo RightElbowServo = null;
    private Servo LeftWristServo = null;
    private Servo RightWristServo = null;
//    private Servo HangLeftForearmServo = null;
//    private Servo HangLeftElbowServo = null;
//    private Servo HangRightForearmServo = null;
//    private Servo HangRightElbowServo = null;
    private Servo Grabber = null;
    private boolean oldCrossPressed = true;
    private boolean oldTrianglePressed = true;
    private boolean oldCirclePressed = true;
    private boolean oldLeftTriggerPressed = false;
    private boolean oldRightTriggerPressed = false;
    private boolean clawIsClosed = true;
    private int index = 0;
    private double[] LEServoPositions = {0.23, 0.21, 0.18, 0.21, 0.40, 0.70, 0.92};
    private double[] REServoPositions = {0.23, 0.21, 0.18, 0.21, 0.40, 0.70, 0.92};
    private double[] LWServoPositions = {0.40, 0.23, 0.20, 0.36, 0.47, 0.20, 0.0};
    private double[] RWServoPositions = {0.40, 0.23, 0.20, 0.36, 0.47, 0.20, 0.0};

    @Override
    public void runOpMode() {
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
        LeftWristServo = hardwareMap.get(Servo.class, "LW");
        RightWristServo = hardwareMap.get(Servo.class, "RW");
//        HangLeftForearmServo = hardwareMap.get(Servo.class, "HLA");
//        HangLeftElbowServo = hardwareMap.get(Servo.class, "HLE");
//        HangRightForearmServo = hardwareMap.get(Servo.class, "HRA");
//        HangRightElbowServo = hardwareMap.get(Servo.class, "HRE");
        Grabber = hardwareMap.get(Servo.class, "G");

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
        LeftWristServo.setDirection(Servo.Direction.FORWARD);
        RightWristServo.setDirection(Servo.Direction.REVERSE);
//        HangLeftForearmServo.setDirection(Servo.Direction.FORWARD);
//        HangLeftElbowServo.setDirection(Servo.Direction.FORWARD);
//        HangRightForearmServo.setDirection(Servo.Direction.FORWARD);
//        HangRightElbowServo.setDirection(Servo.Direction.REVERSE);
        Grabber.setDirection(Servo.Direction.FORWARD);

        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        LeftElbowServo.setPosition(0.5);
//        RightElbowServo.setPosition(0.5);
//        LeftWristServo.setPosition(0.5);
//        RightWristServo.setPosition(0.5);
        Grabber.setPosition(.55);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double leftPower;
            double rightPower;

            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0);
            rightPower   = Range.clip(drive - turn, -1.0, 1.0);

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
                FLMotor.setPower(leftPower);
                FRMotor.setPower(rightPower);
                BLMotor.setPower(leftPower);
                BRMotor.setPower(rightPower);
            }

            IntakeMotor.setPower(gamepad2.dpad_up ? 1 : gamepad2.dpad_down ? -1 : 0);
            LeftSlide.setPower(gamepad2.left_stick_y);
            RightSlide.setPower(gamepad2.left_stick_y);

//            boolean circlePressed = gamepad2.circle;
//            if (circlePressed && !oldCirclePressed) {
//                LeftElbowServo.setPosition(LeftElbowServo.getPosition() == .93 ? .2 : .93);
//                RightElbowServo.setPosition(RightElbowServo.getPosition() == .93 ? .2 : .93);
//            }
//
//            boolean trianglePressed = gamepad2.triangle;
//            if (trianglePressed && !oldTrianglePressed) {
//                LeftWristServo.setPosition(LeftWristServo.getPosition() == 0.22 ? 0 : 0.22);
//                RightWristServo.setPosition(RightWristServo.getPosition() == 0.22 ? 0 : 0.22);
//            }

            boolean crossPressed = gamepad2.cross;
            if (crossPressed && !oldCrossPressed) {
                LeftElbowServo.setPosition(LEServoPositions[index]);
                RightElbowServo.setPosition(REServoPositions[index]);
                LeftWristServo.setPosition(LWServoPositions[index]);
                RightWristServo.setPosition(RWServoPositions[index]);
                index++;
                index = index % LEServoPositions.length;
            }

            // ffffffffffffffffffffffffftttttttttttttttttfffffffffttttt
            Grabber.setPosition(Grabber.getPosition());

            boolean leftTriggerPressed = gamepad2.left_bumper;
            if (leftTriggerPressed && !oldLeftTriggerPressed) {
                Grabber.setPosition(Grabber.getPosition() + 0.02);
            }

            boolean rightTriggerPressed = gamepad2.right_bumper;
            if (rightTriggerPressed && !oldRightTriggerPressed) {
                Grabber.setPosition(Grabber.getPosition() - 0.02);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Bumpers", "(%b), (%b)", gamepad1.left_bumper, gamepad1.right_bumper);
            telemetry.addData("Intake", gamepad2.dpad_up ? 1 : gamepad2.dpad_down ? -1 : 0);
            telemetry.addData("Slides", gamepad2.left_stick_y);
//            telemetry.addData("GrabberPressed", "(%b), (%b)", gamepad2.cross, oldCrossPressed);
//            telemetry.addData("Claw", clawIsClosed);
            telemetry.addData("LeftElbowServoPosition", LeftElbowServo.getPosition());
            telemetry.addData("RightElbowServoPosition", RightElbowServo.getPosition());
            telemetry.addData("LeftWristServoPosition", LeftWristServo.getPosition());
            telemetry.addData("RightWristServoPosition", RightWristServo.getPosition());
            telemetry.addData("Index", index);
            telemetry.addData("Grabber", Grabber.getPosition());
            telemetry.update();
//            oldTrianglePressed = trianglePressed;
//            oldCirclePressed = circlePressed;
            oldCrossPressed = crossPressed;
            oldLeftTriggerPressed = leftTriggerPressed;
            oldRightTriggerPressed = rightTriggerPressed;
        }
    }
}
