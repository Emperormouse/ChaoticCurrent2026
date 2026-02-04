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
public class AutoBLUE extends LinearOpMode {
    MecanumDrive drive;
    Bot bot;
    Pose2d launchPose = new Pose2d(-24.7, -17, Math.toRadians(50));
    Vector2d launchVec = new Vector2d(-24.7, -17);

    public void waitSeconds(double time) {
        long startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() < startTime + time*1000);
    }

    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(-69, -44.44, Math.toRadians(41.3)));
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

            //GRAB SECOND 3 BALLS
            bot.moveToContinuous(new Pose2d(-16, -12, toRadians(-90))),
            bot.intake.setPower(-1.0),
            new EndAfterEitherParallel(
                new Wait(1.3),
                bot.moveToImprecise(new Pose2d(-16, -55, toRadians(-90)), 1.0)
            ),
            bot.stopAction(),
            new Wait(0.3),

            //HIT LEVER

            bot.moveRelativeAction(0.4, 0, -1.0, 1.0),
            new Wait(0.67),

            bot.stopAction(),
            new Wait(0.3),

            /*new EndAfterEitherParallel(
                new EndAfterEitherParallel(
                    bot.waitUntilSeeTag(),
                    new Wait(1.0)
                ),
                bot.moveFieldCentricAction(-1.0, 1.0, -1.0, 1.0)
            ),*/

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
            shootSequence(launchVec),
            bot.canon.setPowerAction(0),


            //GRAB THIRD 3 BALLS

            bot.moveToContinuous(new Pose2d(8.45, -4, toRadians(-87))),
            bot.intake.setPower(-1.0),
            new EndAfterEitherParallel(
                new Wait(1.3),
                bot.moveTo(new Pose2d(9.1, -57, toRadians(-87)), 1.0)
            ),
            bot.stopAction(),
            new Wait(0.5),

            bot.moveRelativeAction(-0.2, -1.0, 0.6, 1.0),
            new EndAfterEitherParallel(
                new Wait(0.8),
                bot.waitUntilSeeTag()
            ),

            bot.stopAction(),

            //SHOOT THIRD 3 BALLS
            bot.intake.setPower(0),
            shootSequence(launchVec),
            bot.canon.setPowerAction(0),


            //GRAB FOURTH 3 BALLS
            bot.moveToContinuous(new Pose2d(29, -10, toRadians(-87))),
            bot.intake.setPower(-1.0),
            new EndAfterEitherParallel(
                new Wait(1.3),
                bot.moveToImprecise(new Pose2d(29, -59.5, toRadians(-87)), 1.0)
            ),
            bot.stopAction(),
            new Wait(0.4),

            bot.moveRelativeAction(-0.2, -1.0, 0.6, 1.0),
            new Wait(0.5),

            //SHOOT FOURTH 3 BALLS
            bot.intake.setPower(0),
            shootSequence(launchVec),
            bot.canon.setPowerAction(0)
        );

        waitForStart();

        //END OF INIT

        Actions.runBlocking(new ParallelAction(
                new SequentialAction(
                    new EndAfterEitherParallel(
                        path,
                        new Wait(29.6)
                    ),
                    bot.intake.setPower(0),

                    bot.moveRelativeAction(-1.0, 0, 0, 1.0),
                    new Wait(0.4),
                    bot.stopAction()
                ),
                bot.telementaryAction(),
                bot.updatePoseUsingAprilTagAction(),
                new KeepRunning(bot.canon.cloneMotorPower())
            )
        );
    }

    public Action shootSequence(Vector2d targetVec) {
        return new EndAfterFirstParallel(
            new SequentialAction(
                bot.moveToTracked(targetVec),
                bot.stopAction(),
                bot.gate.open(),
                new EndAfterFirstParallel(
                    new Wait(2.4),
                    new ParallelAction(
                        new KeepRunning(bot.aimAtGoal()),
                        new SequentialAction(
                            new Wait(0.6),
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

