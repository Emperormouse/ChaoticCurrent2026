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
public class AutoRED extends LinearOpMode {
    MecanumDrive drive;
    Bot bot;
    Pose2d launchPose = new Pose2d(-22.4, 16.6, Math.toRadians(-46));
    Vector2d launchVec = new Vector2d(-23.5, 22.5);

    public void waitSeconds(double time) {
        long startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() < startTime + time*1000);
    }

    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(-69, -44.44, Math.toRadians(41.3)));
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
        bot.isOpModeRunning = true;

        Action path = new SequentialAction(
            //SHOOT FIRST 3 BALLS

            shootSequence(launchVec),
            bot.canon.setPowerAction(0),
            bot.intake.setPower(1.0),

            bot.disableAprilTag(),

            //GRAB SECOND 3 BALLS
            new EndAfterFirstParallel(
                new Wait(1.1),
                new KeepRunning(bot.moveToContinuous(new Pose2d(-13.9, 14, toRadians(85))))
            ),

            bot.intake.setPower(-1.0),
            new EndAfterEitherParallel(
                new Wait(1.2),
                bot.moveToImprecise(new Pose2d(-13.9, 56, toRadians(85)), 1.0)
            ),
            bot.stopAction(),
            new Wait(0.4),
            bot.intake.setPower(0),

            //Lever version 2
            new EndAfterEitherParallel(
                new Wait(1.1),
                new KeepRunning(bot.moveTo(new Pose2d(-7.0, 48, Math.toRadians(5)), 0.7))
                //new KeepRunning(bot.moveTo(new Pose2d(-6.2, 60, Math.toRadians(181)), 0.75, -1))
            ),
            bot.enableAprilTag(),
            bot.moveRelativeAction(1.0, 0, 0, 1.0),
            new Wait(0.4),
            bot.stopAction(),
            new Wait(0.2),

            //SHOOT SECOND 3 BALLS
            shootSequence(launchVec),
            bot.canon.setPowerAction(0),
            bot.intake.setPower(1.0),

            bot.disableAprilTag(),

            //GRAB THIRD 3 BALLS
            bot.moveToContinuous(new Pose2d(11.5, 12, toRadians(85))),
            bot.intake.setPower(-1.0),
            new EndAfterEitherParallel(
                new Wait(1.4),
                bot.moveTo(new Pose2d(11.5, 58.5, toRadians(85)), 1.0)
            ),
            bot.stopAction(),
            new Wait(0.4),
            bot.intake.setPower(0),

            bot.enableAprilTag(),

            bot.moveRelativeAction(0, -0.9, -0.7, 1.0),
            new EndAfterEitherParallel(
                new Wait(1.0),
                bot.waitUntilSeeTag()
            ),

            //SHOOT THIRD 3 BALLS
            shootSequence(launchVec),
            bot.canon.setPowerAction(0),
            bot.intake.setPower(1.0),

            bot.disableAprilTag(),

            //GRAB FOURTH 3 BALLS
            bot.intake.setPower(-1.0),
            bot.moveToContinuous(new Pose2d(33.7, 6, toRadians(85))),
            new EndAfterEitherParallel(
                new Wait(1.3),
                bot.moveToImprecise(new Pose2d(33.7, 58, toRadians(85)), 1.0)
            ),
            bot.stopAction(),
            new Wait(0.3),
            bot.intake.setPower(0),

            bot.enableAprilTag(),

            bot.moveRelativeAction(0, -1.0, -0.8, 1.0),
            new EndAfterEitherParallel(
                new Wait(0.5),
                bot.waitUntilSeeTag()
            ),

            //SHOOT FOURTH 3 BALLS
            shootSequence(launchVec),
            bot.canon.setPowerAction(0),

            bot.intake.setPower(0),
            bot.stopAction()
        );

        waitForStart();

        //END OF INIT

        Actions.runBlocking(new ParallelAction(
                new SequentialAction(
                    new EndAfterEitherParallel(
                        path,
                        new Wait(29.5)
                    ),
                    bot.intake.setPower(0),

                    bot.moveRelativeAction(1.0, 0, 0, 1.0),
                    new Wait(0.5),
                    bot.stopAction()
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













/* package org.firstinspires.ftc.teamcode.code.Auto;

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
import org.firstinspires.ftc.teamcode.code.utility.Actions.KeepRunning;
import org.firstinspires.ftc.teamcode.code.utility.Actions.Wait;
import org.firstinspires.ftc.teamcode.code.utility.Op;
import org.firstinspires.ftc.teamcode.code.utility.Side;

@Autonomous
public class AutoBLUE extends LinearOpMode {
    MecanumDrive drive;
    Bot bot;

    public void waitSeconds(double time) {
        long startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() < startTime + time*1000);
    }

    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(-49.5, -50, Math.toRadians(56)));
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
            new EndAfterEitherParallel(
                new Wait(3.0),
                bot.moveToLaunchSubArc()
            ),
            new Wait(0.5),
            bot.shootClose(Op.AUTO),
            bot.canon.setPowerAction(0),

            //GRAB SECOND 3 BALLS
            bot.intake.setPower(-1.0),
            bot.moveToImprecise(new Pose2d(-11, -12, toRadians(-90))),
            new EndAfterEitherParallel(
                new Wait(2.3),
                bot.moveToImprecise(new Pose2d(-11, -53, toRadians(-90)), 1.0)
            ),
            new Wait(0.6),
            bot.intake.setPower(0),

            //SHOOT SECOND 3 BALLS
            new EndAfterEitherParallel(
                new Wait(4.5),
                bot.moveToLaunchSubArc()
            ),
            new Wait(0.5),
            bot.shootClose(Op.AUTO),
            bot.canon.setPowerAction(0),

            //GRAB THIRD 3 BALLS
            bot.intake.setPower(-1.0),
            bot.moveToImprecise(new Pose2d(32.5, -6, toRadians(-85))),
            new EndAfterEitherParallel(
                new Wait(2.3),
                bot.moveToImprecise(new Pose2d(32.5, -57.25, toRadians(-85)), 1.0)
            ),
            new Wait(0.5),
            bot.intake.setPower(0),

            //SHOOT THIRD 3 BALLS
            new EndAfterEitherParallel(
                new Wait(5),
                bot.moveToLaunchSubArc()
            ),
            new Wait(0.5),
            bot.shootClose(Op.AUTO),
            bot.canon.setPowerAction(0)
        );

        waitForStart();

        //END OF INIT

        Actions.runBlocking(new ParallelAction(
                path,
                bot.updatePoseUsingAprilTagAction()
            )
        );
    }
}
*/