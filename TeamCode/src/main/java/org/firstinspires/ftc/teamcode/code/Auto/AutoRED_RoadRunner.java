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
public class AutoRED_RoadRunner extends LinearOpMode {
    MecanumDrive drive;
    Bot bot;
    Pose2d launchPose = new Pose2d(-23.8, 21, Math.toRadians(-51));
    Vector2d launchVec = new Vector2d(launchPose.position.x, launchPose.position.y);

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

        Action RRPath = bot.drive.actionBuilder(startPos)
            .strafeToLinearHeading(launchVec, toRadians(-45))
            .afterTime(0, aimSequence())
            .waitSeconds(2.4)

            .afterTime(0.3, bot.intake.setPower(-1.0))
            .turnTo(toRadians(60))
            .splineTo(new Vector2d(-12.3, 53), toRadians(90))
            .waitSeconds(0.5)
            .afterTime(0, bot.intake.setPower(0))
            .strafeToLinearHeading(launchVec, toRadians(-45))
            .afterTime(0, aimSequence())
            .waitSeconds(2.4)

            .afterTime(0.5, bot.intake.setPower(-1.0))
            .setTangent(toRadians(-10))
            .splineToSplineHeading(new Pose2d(13, 40, toRadians(90)), toRadians(90))
            .strafeToConstantHeading(new Vector2d(13, 58.5))
            .waitSeconds(0.5)
            .setTangent(toRadians(-90))
            .afterTime(0, bot.intake.setPower(0))
            .splineToLinearHeading(launchPose, toRadians(180))
            .afterTime(0, aimSequence())
            .waitSeconds(2.4)

            .afterTime(0.5, bot.intake.setPower(-1.0))
            .setTangent(toRadians(-10))
            .splineToSplineHeading(new Pose2d(36, 45, toRadians(90)), toRadians(90))
            .strafeToConstantHeading(new Vector2d(36, 58.5))
            .waitSeconds(0.5)
            .setTangent(toRadians(-90))
            .afterTime(0, bot.intake.setPower(0))
            .splineToLinearHeading(launchPose, toRadians(180))
            .afterTime(0, aimSequence())
            .waitSeconds(2.4)
            .build();


        waitForStart();

        //END OF INIT

        Actions.runBlocking(new ParallelAction(
            RRPath,
            new KeepRunning(bot.canon.setVelByDistance(1900)),
            new KeepRunning(bot.updatePoseUsingAprilTagAction()),
            new KeepRunning(bot.canon.cloneMotorPower())
        ));
    }

    public Action aimSequence() {
        return aimSequence(2.4, 0.4);
    }
    public Action aimSequence(double time1, double time2) {
        return new SequentialAction(
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
            bot.intake.setPower(0),
            bot.gate.close(),
            bot.disableAprilTag()
        );
    }
}


