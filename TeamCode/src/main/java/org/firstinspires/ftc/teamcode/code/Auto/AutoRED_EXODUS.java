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
public class AutoRED_EXODUS extends LinearOpMode {
    MecanumDrive drive;
    Bot bot;
    Pose2d launchPose = new Pose2d(-21.4, 15.6, Math.toRadians(-46));
    Vector2d launchVec = new Vector2d(-24.7, 23.4);

    public void waitSeconds(double time) {
        long startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() < startTime + time*1000);
    }

    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(-52.7, -46.0, Math.toRadians(-90)));
        bot = new Bot(hardwareMap, drive, Side.RED, telemetry);

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
            bot.intake.setPower(1.0),

            //GRAB SECOND 3 BALLS
            new EndAfterFirstParallel(
                new Wait(1.1),
                new KeepRunning(bot.moveToContinuous(new Pose2d(-14.5, 14, toRadians(90))))
            ),

            bot.intake.setPower(-1.0),
            new EndAfterEitherParallel(
                new Wait(1.2),
                bot.moveToImprecise(new Pose2d(-14.5, 53.5, toRadians(90)), 1.0)
            ),
            bot.stopAction(),
            new Wait(0.3),
            bot.intake.setPower(0),

            //Lever version 2
            new EndAfterEitherParallel(
                new Wait(1.7),
                //bot.moveTo(new Pose2d(-9.8, 50, Math.toRadians(0)), 0.9)
                new KeepRunning(bot.moveTo(new Pose2d(-6.2, 60, Math.toRadians(181)), 0.75, -1))
            ),

            //SHOOT SECOND 3 BALLS
            shootSequence(launchVec),
            bot.canon.setPowerAction(0),
            bot.intake.setPower(1.0),

            //GRAB THIRD 3 BALLS
            bot.moveToContinuous(new Pose2d(10.0, 12, toRadians(85))),
            bot.intake.setPower(-1.0),
            new EndAfterEitherParallel(
                new Wait(1.4),
                bot.moveTo(new Pose2d(10.0, 58, toRadians(85)), 1.0)
            ),
            bot.stopAction(),
            new Wait(0.3),

            /*bot.moveRelativeAction(0.4, 0, -1.0, 1.0),
            new Wait(0.6),
            bot.stopAction(),
            new Wait(0.5),*/

            new EndAfterEitherParallel(
                new Wait(100.3),
                new KeepRunning(bot.moveTo(new Pose2d(-1.7, 56.5, 0)))
            ),

            //SHOOT THIRD 3 BALLS
            shootSequence(launchVec),
            bot.canon.setPowerAction(0),

            //HIT LEVER AGAIN
            new EndAfterEitherParallel(
                new Wait(2.5),
                bot.moveToImprecise(new Pose2d(-3, 56.0, 0))
            ),
            new Wait(0.5),
            bot.moveToImprecise(new Pose2d(-3, 46.0, 0)),

            bot.intake.setPower(0),
            bot.stopAction()
        );

        waitForStart();

        //END OF INIT

        Actions.runBlocking(new ParallelAction(
                new SequentialAction(
                    new EndAfterFirstParallel(
                        new Wait(25.5),
                        path
                    ),
                    bot.intake.setPower(0),
                    new EndAfterEitherParallel(
                        new Wait(3.9),
                        shootSequence(launchVec, 3.9, 0.5)
                    ),
                    bot.moveRelativeAction(1.0, 0, 0, 1.0),
                    new Wait(0.6),
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
        return shootSequence(targetVec, 2.5, 0.5);
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
