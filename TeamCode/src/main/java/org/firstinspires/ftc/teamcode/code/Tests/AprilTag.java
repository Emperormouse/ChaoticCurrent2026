package org.firstinspires.ftc.teamcode.code.Tests;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.code.Subsystems.Bot;
import org.firstinspires.ftc.teamcode.code.utility.Op;
import org.firstinspires.ftc.teamcode.code.utility.Side;

@TeleOp
public class AprilTag extends LinearOpMode {
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Bot bot = new Bot(hardwareMap, drive, Side.BLUE, telemetry);
        bot.initialize();

        waitForStart();

        while(opModeIsActive()) {

        }
    }
}
