package org.firstinspires.ftc.teamcode.disabled;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name="custom name lol")
public class KyleLikesLearning extends OpMode {
    @Override
    public void init() {
        telemetry.addData("Hello", "World");
    }
    @Override
    public void loop() {
        // LETS GO KYLE!!!!!!
    }
}
