package org.firstinspires.ftc.teamcode.code.Tests;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.code.Subsystems.Canon;

@Disabled
@TeleOp
public class MotorTest extends LinearOpMode {
    public void runOpMode() {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "motor1");
        Canon canon = new Canon(hardwareMap);

        long start = System.currentTimeMillis();
        Actions.runBlocking(canon.spinUp(1800));

        double timeToSpeed = (double)(System.currentTimeMillis() - start) / 1000;

        Action a = canon.maintainSpeed(1800);
        while(!isStopRequested()) {
            a.run(null);

            telemetry.addData("Speed: ", motor.getVelocity());
            telemetry.addData("Time To Spin Up: ", timeToSpeed);
            telemetry.update();
        }
    }
}
