///* Copyright (c) 2017 FIRST. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright notice, this
// * list of conditions and the following disclaimer in the documentation and/or
// * other materials provided with the distribution.
// *
// * Neither the name of FIRST nor the names of its contributors may be used to endorse or
// * promote products derived from this software without specific prior written permission.
// *
// * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
//
//package org.firstinspires.ftc.teamcode.test;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.PoseVelocity2d;
//import com.acmerobotics.roadrunner.TimeTrajectory;
//import com.acmerobotics.roadrunner.Trajectory;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.constants.TeleopServoConstants;
//import org.firstinspires.ftc.teamcode.rr.MecanumDrive;
//
//import java.util.Timer;
//import java.util.TimerTask;
//
//
///*
// * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
// * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
// * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
// * class is instantiated on the Robot Controller and executed.
// *
// * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
// * It includes all the skeletal structure that all linear OpModes contain.
// *
// * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
// * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
// */
//
//@TeleOp(name="TestTeleop2", group="Robot")
//public class KongTeleopTest2 extends LinearOpMode {
//    MecanumDrive drive;
//    TrajectoryActionBuilder aB;
//    public static boolean isArmMoving = false;
//    public static boolean isPokerMoving = false;
//    public static boolean isRobotInTrajectory = false;
//    // Declare OpMode members.
//    private ElapsedTime runtime = new ElapsedTime();
//    private Timer timer = new Timer();
//    private DcMotor FLMotor = null;
//    private DcMotor FRMotor = null;
//    private DcMotor BLMotor = null;
//    private DcMotor BRMotor = null;
//    private DcMotor IntakeMotor = null;
//    private DcMotor LeftSlide = null;
//    private DcMotor RightSlide = null;
//    private Servo LeftElbowServo = null;
//    private Servo RightElbowServo = null;
//    private Servo Poker = null;
//    private Servo RightWristServo = null;
//    private Servo Ringer = null;
//    private Servo PlaneLauncher = null;
//    private boolean oldCrossPressed = true;
//    private boolean oldTrianglePressed = true;
//    private boolean oldCirclePressed = true;
//    private boolean oldSquarePressed = true;
//    private boolean oldLBumper = true;
//    private boolean clawIsClosed = true;
//    private int index = 0;
//    private double[] LEServoPositions = TeleopServoConstants.LEServoPositions;
//    private double[] REServoPositions = TeleopServoConstants.REServoPositions;
//    private double[] PokerPositions = TeleopServoConstants.PokerPositions;
//    private double[] RWServoPositions = TeleopServoConstants.RWServoPositions;
//    private double[] RingerPositions = TeleopServoConstants.RingerPositions;
//
//    private final int DELAY_BETWEEN_MOVES = 100;
//
//    public class CancelableFollowTrajectoryAction implements Action {
//        private final MecanumDrive.FollowTrajectoryAction action;
//        private boolean cancelled = false;
//
//        public CancelableFollowTrajectoryAction(TimeTrajectory t) {
//            action = new MecanumDrive.FollowTrajectoryAction(t);
//        }
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            if (cancelled) {
//                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
//                return false;
//            }
//
//            return action.run(telemetryPacket);
//        }
//
//        public void cancelAbruptly() {
//            cancelled = true;
//        }
//    }
//    @Override
//    public void runOpMode() throws InterruptedException {
//        FtcDashboard dash = FtcDashboard.getInstance();
//
//        TimeTrajectory traj = new TimeTrajectory();
//
//        CancelableFollowTrajectoryAction cancelableAction = new CancelableFollowTrajectoryAction(traj);
//        while (opModeIsActive()) {
//            TelemetryPacket packet = new TelemetryPacket();
//            cancelableAction.preview(packet.fieldOverlay());
//            if (!cancelableAction.run(packet)) {
//                break;
//            }
//
//            if (gamepad1.a) {
//                cancelableAction.cancelAbruptly();
//            }
//
//            dash.sendTelemetryPacket(packet);
//        }
//    }
//}
