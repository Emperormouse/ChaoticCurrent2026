package org.firstinspires.ftc.teamcode.code.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Example1_Motor extends LinearOpMode {
    //This opMode sets a motor to full speed forward if the 'a' button is pressed, full speed backwards
    //if the 'b' button is pressed, and otherwise a speed determined by the right analog stick.
    public void runOpMode() {
        //Setup
        //The variable name can be anything, but the string must be the name that is assigned to it on the control hub.
        DcMotor motor1 = hardwareMap.get(DcMotor.class, "motor1");

        //Main Loop, which stops when the stop button on the driver hub is pressed
        while(opModeIsActive()) {
            if (gamepad1.a) { //This checks if the 'a' button is pressed
                motor1.setPower(1.0); //Motors' speed can be set to any value from -1.0 to 1.0
            }
            else if (gamepad1.b) { //If a is not pressed, this checks if the 'b' button is pressed
                motor1.setPower(-1.0);
            }
            else { // If neither are pressed, this is run
                //The analog sticks each have an x and y value between -1.0 and 1.0 representing their position
                //This is using the right stick's y-value.
                motor1.setPower(gamepad1.right_stick_y);
            }
        }

    }
}

