package org.firstinspires.ftc.teamcode.code.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous
public class MotorSpeed extends LinearOpMode {
    public void runOpMode() {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "motor1");

        while(!isStarted());

        while (opModeIsActive()) {
            motor.setPower(0.7);
            
            telemetry.addData("Speed: ", motor.getVelocity());
            telemetry.update();
        }
    }
}
