package org.firstinspires.ftc.teamcode.examples.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

//This opMode turns a servo to the left or right side of its limit, based on whether a or b was pressed.
public class Example2_Servo extends LinearOpMode {
    public void runOpMode() {
        //Setup
        //The variable name can be anything, but the string must be the name that is assigned to it on the control hub.
        Servo servo1 = hardwareMap.get(Servo.class, "servo1");
        servo1.setPosition(0.5); //The sets the servo to the halfway point at the start

        //Main Loop, which stops when the stop button on the driver hub is pressed
        while(opModeIsActive()) {
            if (gamepad1.a) { //This checks if the 'a' button is pressed
                servo1.setPosition(0.0); //This sets the position of the servo, on a range of 0-1.
                //It usually has a range of 180 degrees or 270 degrees.
                //The servo will automatically turn to that position.
            }
            else if (gamepad1.b) { //If a is not pressed, this checks if the 'b' button is pressed
                servo1.setPosition(1.0);
            }
            //Even after a and b are released, the servo will stay at either 0.0 or 1.0, instead of
            //going back to 0.5, since no code is telling it to go back.
            //Also, the speed of servos cannot be set, only their target position
        }

    }
}
