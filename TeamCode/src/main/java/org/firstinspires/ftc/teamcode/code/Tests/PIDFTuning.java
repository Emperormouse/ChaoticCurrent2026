package org.firstinspires.ftc.teamcode.code.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class PIDFTuning extends LinearOpMode {
    double p = 1.0;
    double f = 1.0;
    double stepSize = 1.0;
    double targetSpeed = 0;

    DcMotorEx motor;
    DcMotor motor2;

    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, "left_launcher");
        motor2 = (DcMotor)hardwareMap.get(DcMotorEx.class, "par");

        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.bWasPressed()) {
                stepSize *= 10;
            }
            if (gamepad1.aWasPressed()) {
                stepSize /= 10;
            }
            if (gamepad1.dpadRightWasPressed()) {
                f += stepSize;
            }
            if (gamepad1.dpadLeftWasPressed()) {
                f -= stepSize;
            }
            if (gamepad1.dpadUpWasPressed()) {
                p += stepSize;
            }
            if (gamepad1.dpadDownWasPressed()) {
                p -= stepSize;
            }

            if (gamepad1.xWasPressed()) {
                targetSpeed = -700;
            }
            if (gamepad1.yWasPressed()) {
                targetSpeed = -1800;
            }
            if (gamepad1.rightBumperWasPressed()) {
                targetSpeed = 0;
            }
            if (gamepad1.leftBumperWasPressed()) {
                targetSpeed = -1000;
            }

            PIDFCoefficients pid = new PIDFCoefficients(p, 0, 0 , f);
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid);

            motor.setVelocity(targetSpeed);
            motor2.setPower(motor.getPower());

            telemetry.update();
            telemetry.addData("Target Velocity: ", targetSpeed);
            telemetry.addData("Current Velocity: ", motor.getVelocity());
            telemetry.addData("Error: ", targetSpeed - motor.getVelocity());
            telemetry.addLine("===================================");
            telemetry.addData("P: ", "%.4f", p);
            telemetry.addData("F: ", "%.4f", f);
            telemetry.addData("Step Size: ", "%.4f", stepSize);
        }
    }


}
