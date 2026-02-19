package org.firstinspires.ftc.teamcode.code.Auto;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
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
    Vector2d launchVec = new Vector2d(-24.7, -17);

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
            shootSequence(launchVec),
            bot.canon.setPowerAction(0),

            bot.disableAprilTag(),

            //GRAB SECOND 3 BALLS
            bot.moveToContinuous(new Pose2d(-16, -12, toRadians(-90))),
            bot.intake.setPower(-1.0),
            new EndAfterEitherParallel(
                new Wait(1.3),
                bot.moveToImprecise(new Pose2d(-16, -55, toRadians(-90)), 1.0)
            ),
            bot.stopAction(),
            new Wait(0.3),

            //Lever version 2
            new EndAfterEitherParallel(
                new Wait(0.9),
                new KeepRunning(bot.moveTo(new Pose2d(-2, -47, 0), 0.7))
                //new KeepRunning(bot.moveTo(new Pose2d(-6.2, 60, Math.toRadians(181)), 0.75, -1))
            ),

            bot.enableAprilTag(),
            bot.intake.setPower(0),

            bot.moveRelativeAction(-1.0, 0, 0, 1.0),
            new Wait(0.4),
            bot.stopAction(),
            new Wait(0.2),

            //SHOOT SECOND 3 BALLS
            shootSequence(launchVec),
            bot.canon.setPowerAction(0),

            bot.disableAprilTag(),

            //GRAB THIRD 3 BALLS

            bot.moveToContinuous(new Pose2d(7.8, -4, toRadians(-90))),
            bot.intake.setPower(-1.0),
            new EndAfterEitherParallel(
                new Wait(1.3),
                bot.moveTo(new Pose2d(7.8, -57, toRadians(-90)), 1.0)
            ),
            bot.stopAction(),
            new Wait(0.5),

            bot.enableAprilTag(),

            new ParallelAction(
                new SequentialAction(
                    new Wait(4.0),
                    bot.intake.setPower(0)
                ),
                new KeepRunning(bot.moveTo(new Pose2d(0, -53.5, 0)))
            )
        );

        waitForStart();

        //END OF INIT

        Actions.runBlocking(new ParallelAction(
                new SequentialAction(
                    new EndAfterFirstParallel(
                        new Wait(25),
                        path
                    ),
                    bot.intake.setPower(0),
                    new EndAfterEitherParallel(
                        new Wait(4.5),
                        shootSequence(launchVec, 4.5, 0.5)
                    ),
                    bot.moveRelativeAction(-1.0, 0, 0, 1.0),
                    new Wait(0.5),
                    bot.stopAction(),
                    bot.canon.setPowerAction(0)
                ),

                bot.telementaryAction(),
                bot.updatePoseUsingAprilTagAction(),
                new KeepRunning(bot.canon.cloneMotorPower())
            )
        );
    }

    public Action shootSequence(Vector2d targetVec) {
        return shootSequence(targetVec, 2.4, 0.4);
    }
    public Action shootSequence(Vector2d targetVec, double time1, double time2) {
        return new EndAfterFirstParallel(
            new SequentialAction(
                bot.moveToTracked(targetVec),
                bot.stopAction(),
                bot.gate.open(),
                new EndAfterFirstParallel(
                    new Wait(time1),
                    new ParallelAction(
                        new KeepRunning(bot.aimAtGoal()),
                        new SequentialAction(
                            new Wait(time2),
                            bot.intake.setPower(-1.0)
                        )
                    )
                ),
                bot.intake.setPower(0),
                bot.gate.close()
            ),
            bot.canon.setVelByDistance()
        );
    }
}
