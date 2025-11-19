package org.firstinspires.ftc.teamcode.code.TeleOp;

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
        Bot bot = new Bot(hardwareMap, drive.localizer, Side.BLUE, telemetry);
        bot.initialize();

        waitForStart();

        while(opModeIsActive()) {
            bot.aprilTagTelementary();
            bot.updatePoseUsingAprilTag();
            telemetry.update();

            double r = bot.turnBasedOnAprilTag(Side.BLUE) - gamepad1.right_stick_x;

            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x * 1.1;
            y /= Math.max(1, Math.abs(r*2));
            x /= Math.max(1, Math.abs(r*2));

            bot.moveFieldCentric(x, y, r, Op.TELE);
        }
    }
}
