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
    Pose2d launchPose = new Pose2d(-12.3, -16, Math.toRadians(45));
    Vector2d launchVec = new Vector2d(launchPose.position.x, launchPose.position.y);
    double launchSpeed = 1409;

    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(-69, -44.44, Math.toRadians(41.3)));
        bot = new Bot(hardwareMap, drive, Side.RED, telemetry);

        bot.initialize();
        long startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() < startTime + 4000) { //Scan for april tag for 4 seconds
            bot.updatePose();
            bot.aprilTagTelementary();
            telemetry.update();
        }

        Pose2d startPos = bot.botPose;
        telemetry.addData("Start Position: ", startPos);
        telemetry.update();
        bot.isOpModeRunning = true;

        Action RRPath = bot.drive.actionBuilder(startPos)
            .afterTime(0, bot.canon.setVelAction(launchSpeed))
            .strafeToLinearHeading(launchVec, toRadians(45))
            //.afterTime(0, aimSequence(2.0, 0.8))
            .waitSeconds(2.0)

            .setTangent(toRadians(10))
            .splineToSplineHeading(new Pose2d(13, -40, toRadians(-90)), toRadians(-90))
            .strafeToConstantHeading(new Vector2d(13, -57.5))
            .waitSeconds(0.2)
            .setTangent(toRadians(90))
            .afterTime(0, bot.canon.setVelAction(launchSpeed))
            .splineToLinearHeading(launchPose, toRadians(180))
            //.afterTime(0, aimSequence())
            .waitSeconds(1.6)

            .setTangent(toRadians(-20))
            .splineToSplineHeading(new Pose2d(11, -50, toRadians(-125)), toRadians(-90))
            .strafeToConstantHeading(new Vector2d(12, -60))
            .waitSeconds(3.5)
            .setTangent(toRadians(90))
            .afterTime(0, bot.canon.setVelAction(launchSpeed))
            .splineToLinearHeading(launchPose, toRadians(180))
            //.afterTime(0, aimSequence())
            .waitSeconds(1.6)

            .setTangent(toRadians(-20))
            .splineToSplineHeading(new Pose2d(11, -50, toRadians(-125)), toRadians(-90))
            .strafeToConstantHeading(new Vector2d(12, -60))
            .waitSeconds(3.5)
            .setTangent(toRadians(90))
            .afterTime(0, bot.canon.setVelAction(launchSpeed))
            .splineToLinearHeading(launchPose, toRadians(180))
            //.afterTime(0, aimSequence())
            .waitSeconds(1.6)

            .turnTo(toRadians(-90))
            .strafeToConstantHeading(new Vector2d(-12.3, -53))
            .waitSeconds(0.2)
            .afterTime(0, bot.canon.setVelAction(launchSpeed))
            .strafeToLinearHeading(launchVec, launchPose.heading.toDouble())
            //.afterTime(0, aimSequence())
            .waitSeconds(1.6)
            .build();

        Action RRPath2 = bot.drive.actionBuilder(startPos)
            .strafeToLinearHeading(launchVec, toRadians(45))
            .waitSeconds(1.2)

            //.setTangent(toRadians(10))
            /*.splineTo(new Vector2d(13, -40), toRadians(-90))
            .strafeToConstantHeading(new Vector2d(13, -57.5))
            .waitSeconds(0.2)
            .setTangent(toRadians(90))
            .strafeToLinearHeading(launchVec, toRadians(45))
            .waitSeconds(1.2)*/

            .strafeToLinearHeading(new Vector2d(9.0, -58), toRadians(-125))
            .waitSeconds(3.5)
            .setTangent(45)
            .strafeToLinearHeading(launchVec, toRadians(45))
            .waitSeconds(1.2)

            .strafeToLinearHeading(new Vector2d(9.0, -58), toRadians(-125))
            .waitSeconds(3.5)
            .setTangent(45)
            .strafeToLinearHeading(launchVec, toRadians(45))
            .waitSeconds(1.2)

            /*.turnTo(toRadians(-90))
            .strafeToConstantHeading(new Vector2d(-12.3, -53))
            .waitSeconds(0.2)
            .strafeToLinearHeading(launchVec, launchPose.heading.toDouble())
            .waitSeconds(1.2)*/
            .build();

        Action RRPath3 = new SequentialAction(
            bot.moveToImprecise(launchPose),
            new Wait(1.6),
            bot.moveToImprecise(new Pose2d(9.0, -58, toRadians(-125))),
            new Wait(1.6),
            bot.moveToImprecise(launchPose),
            new Wait(1.6),
            bot.moveToImprecise(new Pose2d(9.0, -58, toRadians(-125))),
            new Wait(1.6),
            bot.moveToImprecise(launchPose)
        );


        waitForStart();

        //END OF INIT

        Actions.runBlocking(new ParallelAction(
            RRPath3,
            new KeepRunning(bot.updatePoseAction()),
            new KeepRunning(bot.canon.cloneMotorPower()),
            new KeepRunning(bot.canon.setVelAction(launchSpeed))
        ));
    }

    public Action aimSequence() {
        return aimSequence(1.6, 0.4);
    }
    public Action aimSequence(double time1, double time2) {
        return new SequentialAction(
            bot.intake.setPower(0.0),
            bot.enableAprilTag(),
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
            bot.canon.setPowerAction(0),
            bot.gate.close(),
            bot.disableAprilTag()
        );
    }
}


