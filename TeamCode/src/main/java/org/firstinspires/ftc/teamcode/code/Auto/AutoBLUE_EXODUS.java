package org.firstinspires.ftc.teamcode.code.Auto;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.code.Subsystems.Bot;
import org.firstinspires.ftc.teamcode.code.utility.Actions.EndAfterEitherParallel;
import org.firstinspires.ftc.teamcode.code.utility.Actions.EndAfterFirstParallel;
import org.firstinspires.ftc.teamcode.code.utility.Actions.KeepRunning;
import org.firstinspires.ftc.teamcode.code.utility.Actions.Wait;
import org.firstinspires.ftc.teamcode.code.utility.Op;
import org.firstinspires.ftc.teamcode.code.utility.Side;

@Autonomous
public class AutoBLUE_EXODUS extends LinearOpMode {
    MecanumDrive drive;
    Bot bot;
    Pose2d launchPose = new Pose2d(-24.7, -17, Math.toRadians(50));

    public void waitSeconds(double time) {
        long startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() < startTime + time*1000);
    }

    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(-52.7, -46.0, Math.toRadians(-90)));
        bot = new Bot(hardwareMap, drive, Side.BLUE, telemetry);

        bot.initialize();
        long startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() < startTime + 5000) { //Scan for april tag for 5 seconds
            bot.updatePoseUsingAprilTag();
            bot.aprilTagTelementary();
            telemetry.update();
        }

        Pose2d startPos = drive.localizer.getPose();
        telemetry.addData("Start Position: ", startPos);
        telemetry.update();

        Action path = new SequentialAction(
            //SHOOT FIRST 3 BALLS

            shootSequence(),
            bot.canon.setPowerAction(0),

            //GRAB SECOND 3 BALLS
            bot.moveToContinuous(new Pose2d(-15.9, -12, toRadians(-90))),
            bot.intake.setPower(-1.0),
            new EndAfterEitherParallel(
                new Wait(1.3),
                bot.moveToImprecise(new Pose2d(-15.9, -53.5, toRadians(-90)), 1.0)
            ),
            bot.stopAction(),
            new Wait(0.3),

            //HIT LEVER 1

            bot.moveRelativeAction(0.4, 0, -0.8, 1.0),
            new Wait(0.67),

            bot.stopAction(),
            new Wait(0.3),

            bot.moveRelativeAction(-1.0, 0.8, 0, 1.0),
            new Wait(0.3),
            bot.moveRelativeAction(0, 0.8, -1.0, 1.0),
            new EndAfterEitherParallel(
                bot.waitUntilSeeTag(),
                new Wait(0.8)
            ),
            bot.intake.setPower(0),
            bot.stopAction(),

            //SHOOT SECOND 3 BALLS
            shootSequence(),
            bot.canon.setPowerAction(0),


            //GRAB THIRD 3 BALLS

            bot.moveToContinuous(new Pose2d(9.0, -6, toRadians(-87))),
            bot.intake.setPower(-1.0),
            new EndAfterEitherParallel(
                new Wait(1.6),
                bot.moveTo(new Pose2d(9.0, -58.5, toRadians(-87)), 1.0)
            ),
            bot.stopAction(),
            new Wait(0.5),

            bot.moveRelativeAction(-0.4, 0, 1.0, 1.0),
            new Wait(0.6),
            bot.stopAction(),
            new Wait(0.5),

            //SHOOT THIRD 3 BALLS
            bot.intake.setPower(0),
            shootSequence(),
            bot.canon.setPowerAction(0),

            bot.moveToImprecise(new Pose2d(-2.5, -53.3, 0)),
            bot.moveRelativeAction(-0.5, 0, 0, 1.0),
            new Wait(0.3),
            bot.stopAction(),
            new Wait(0.5),
            bot.moveRelativeAction(0.5, 0, 0, 1.0),
            new Wait(0.3),
            bot.intake.setPower(0),
            bot.stopAction()
        );

        waitForStart();

        //END OF INIT

        Actions.runBlocking(new ParallelAction(
                path,
                bot.telementaryAction(),
                bot.updatePoseUsingAprilTagAction(),
                new KeepRunning(bot.canon.cloneMotorPower())
            )
        );
    }

    public Action shootSequence() {
        return new SequentialAction(
            bot.canon.setVelAction(bot.canon.CLOSE_SPEED),
            bot.moveToVeryImprecise(launchPose),

            new EndAfterFirstParallel(
                bot.shootClose(Op.AUTO),
                new SequentialAction(
                    new EndAfterFirstParallel(
                        new Wait(0.8),
                        new KeepRunning(bot.moveToLaunchSubArc())
                    ),
                    bot.stopAction()
                )
            ),
            bot.gate.close(),
            bot.intake.setPower(1.0)
        );

    }
}
