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

@TeleOp(name="TestServo", group="Robot")
public class AAAAA extends LinearOpMode {
    private Servo LeftElbowServo = null;
    private Servo RightElbowServo = null;
    private Servo LeftWristServo = null;
    private Servo RightWristServo = null;
    private boolean oldCrossPressed = false;
    private boolean oldTrianglePressed = false;
    private boolean oldDPAD_UPPressed = false;
    private boolean oldDPAD_DOWNPressed = false;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        LeftElbowServo = hardwareMap.get(Servo.class, "LE");
        RightElbowServo = hardwareMap.get(Servo.class, "RE");
        LeftWristServo = hardwareMap.get(Servo.class, "LW");
        RightWristServo = hardwareMap.get(Servo.class, "RW");

        LeftElbowServo.setDirection(Servo.Direction.FORWARD);
        RightElbowServo.setDirection(Servo.Direction.REVERSE);
        LeftWristServo.setDirection(Servo.Direction.FORWARD);
        RightWristServo.setDirection(Servo.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        LeftElbowServo.setPosition(0);
        RightElbowServo.setPosition(0);
        LeftWristServo.setPosition(0);
        RightWristServo.setPosition(0);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // ffffffffffffffffffffffffftttttttttttttttttfffffffffttttt
            boolean crossPressed = gamepad2.cross;
            if (crossPressed && !oldCrossPressed) {
                LeftElbowServo.setPosition(Range.clip(LeftElbowServo.getPosition() + 0.01, 0.0, 1.0));
                RightElbowServo.setPosition(Range.clip(RightElbowServo.getPosition() + 0.01, 0.0, 1.0));
            }

            boolean trianglePressed = gamepad2.triangle;
            if (trianglePressed && !oldTrianglePressed) {
                LeftElbowServo.setPosition(Range.clip(LeftElbowServo.getPosition() - 0.01, 0.0, 1.0));
                RightElbowServo.setPosition(Range.clip(RightElbowServo.getPosition() - 0.01, 0.0, 1.0));
            }

            boolean dpad_up = gamepad2.dpad_up;
            if (dpad_up && !oldDPAD_UPPressed) {
                LeftWristServo.setPosition(Range.clip(LeftWristServo.getPosition() + 0.01, 0.0, 1.0));
                RightWristServo.setPosition(Range.clip(RightWristServo.getPosition() + 0.01, 0.0, 1.0));
            }

            boolean dpad_down = gamepad2.dpad_down;
            if (dpad_down && !oldDPAD_DOWNPressed) {
                LeftWristServo.setPosition(Range.clip(LeftWristServo.getPosition() - 0.01, 0.0, 1.0));
                RightWristServo.setPosition(Range.clip(RightWristServo.getPosition() - 0.01, 0.0, 1.0));
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("LeftElbowServoPosition", LeftElbowServo.getPosition());
            telemetry.addData("RightElbowServoPosition", RightElbowServo.getPosition());
            telemetry.addData("LeftWristServoPosition", LeftWristServo.getPosition());
            telemetry.addData("RightWristServoPosition", RightWristServo.getPosition());

            telemetry.update();
            oldCrossPressed = crossPressed;
            oldTrianglePressed = trianglePressed;
            oldDPAD_UPPressed = dpad_up;
            oldDPAD_DOWNPressed = dpad_down;
        }
    }
}
