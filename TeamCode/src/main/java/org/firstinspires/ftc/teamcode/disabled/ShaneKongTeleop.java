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

package org.firstinspires.ftc.teamcode.disabled;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.teamcode.constants.TeleopServoConstants;

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

@TeleOp(name="ShaneKongTeleop", group="Test")
@Disabled
public class ShaneKongTeleop extends LinearOpMode {
    public static boolean isPokerMoving = false;
    private Servo Poker = null;
    private boolean oldCrossPressed = true;
    private boolean oldTrianglePressed = true;
    private boolean oldCirclePressed = true;
    private boolean oldSquarePressed = true;
    private boolean oldLBumper = true;
    private double[] PokerPositions = TeleopServoConstantsSH.PokerPositions;
    private Timer timer = new Timer();
    private int index = 0;

    @Override
    public void runOpMode() {

        class setIsPokerMoving extends TimerTask {
            boolean val;
            public setIsPokerMoving(boolean v) {
                this.val = v;
            }
            public void run() {
                isPokerMoving = val;
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

        telemetry.addData("Status", "sInitialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        Poker = hardwareMap.get(Servo.class, "P");

        Poker.setDirection(Servo.Direction.FORWARD);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        new PutPokerToCertainPosition(0).run();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // ffffffffffffffffffffffffftttttttttttttttttfffffffffttttt
            boolean crossPressed = gamepad2.cross;
            if (crossPressed && !oldCrossPressed) {
                new PutPokerToCertainPosition(++index).run();
            }

            telemetry.update();
            oldCrossPressed = crossPressed;
        }
    }
}
