package org.firstinspires.ftc.teamcode.code.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class OdoTest extends LinearOpMode {
    public void runOpMode() {
        DcMotorEx par = hardwareMap.get(DcMotorEx.class, "par");
        DcMotorEx perp = hardwareMap.get(DcMotorEx.class, "perp");

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("par_pos ", par.getCurrentPosition());
            telemetry.addData("perp_pos: ", perp.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("par_vel ", par.getVelocity());
            telemetry.addData("perp_vel: ", perp.getVelocity());
            telemetry.update();
        }
    }
}
