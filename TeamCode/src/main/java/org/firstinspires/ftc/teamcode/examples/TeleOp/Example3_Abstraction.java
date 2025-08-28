package org.firstinspires.ftc.teamcode.examples.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.examples.subsystems.Claw;

// This code uses the "Claw" class which I created in the subsystems folder, to "abstract" some code sway
// it sets a servo to one of two positions, based on whether a or b is pressed. It could be used
// for a simple claw.
public class Example3_Abstraction extends LinearOpMode {
    public void runOpMode() {
        //Setup
        Claw claw = new Claw(hardwareMap);

        while (opModeIsActive()) {
            if (gamepad1.a) {
                claw.open();
            } else if (gamepad1.b) {
                claw.close();
            }
        }
    }
}
