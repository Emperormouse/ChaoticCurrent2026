package org.firstinspires.ftc.teamcode.code.Auto;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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
@Config
public class AutoRED extends LinearOpMode {
    public static class PARAMS {
        public double x1 = 8.7;
        public double angle = 120;
        public double y1 = 45;
        public double y2 = 61.5;
        public double xMiddle = 15.5;
        public double yMiddle1 = 30;
        public double yMiddle2 = 62;
        public double closeX = -13.25;
        public double closeY1 = 26;
        public double closeY2 = 56;
        public double launchX = -15;
        public double launchY = 17;
        public double launchR = -44;
        public double lenBack = 5;
    }
    public static PARAMS PARAMS = new PARAMS();

    MecanumDrive drive;
    Bot bot;
    Pose2d launchPose = new Pose2d(PARAMS.launchX, PARAMS.launchY, Math.toRadians(PARAMS.launchR));
    Vector2d launchVec = new Vector2d(launchPose.position.x, launchPose.position.y);
    double launchSpeed = 1440;

    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(-52.87, -36.2, Math.toRadians(-90)));
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
            .strafeToSplineHeading(launchVec, toRadians(PARAMS.launchR+6))
            .afterTime(0, aimSequence(2.3, 1.1))
            .waitSeconds(2.3)

            .afterTime(0, bot.intake.setPower(-1.0))
            .setTangent(toRadians(0))
            .splineToSplineHeading(new Pose2d(PARAMS.xMiddle, PARAMS.yMiddle1, toRadians(90)), toRadians(90))
            .strafeToConstantHeading(new Vector2d(PARAMS.xMiddle, PARAMS.yMiddle2))
            .afterTime(0.3, bot.intake.setPower(0))
            .setTangent(toRadians(-90))
            .splineToSplineHeading(launchPose, toRadians(180))
            .afterTime(0, aimSequence())
            .waitSeconds(1.2)

            .afterTime(0, bot.intake.setPower(-1.0))
            .setTangent(toRadians(0))
            //.splineToSplineHeading(new Pose2d(PARAMS.x1, PARAMS.y2, toRadians(PARAMS.angle)), toRadians(120))
            .splineToSplineHeading(new Pose2d(PARAMS.x1+PARAMS.lenBack*cos(toRadians(60)), PARAMS.y2-PARAMS.lenBack*sin(toRadians(60)), toRadians(PARAMS.angle)), toRadians(120))
            .strafeToLinearHeading(new Vector2d(PARAMS.x1, PARAMS.y2), toRadians(PARAMS.angle))
            .waitSeconds(1.5)
            .afterTime(0.3, bot.intake.setPower(0))
            .setTangent(toRadians(-90))
            .splineToSplineHeading(launchPose, toRadians(180))
            .afterTime(0, aimSequence())
            .waitSeconds(1.2)

            .afterTime(0, bot.intake.setPower(-1.0))
            .setTangent(toRadians(0))
            //.splineToSplineHeading(new Pose2d(PARAMS.x1, PARAMS.y2, toRadians(PARAMS.angle)), toRadians(120))
            .splineToSplineHeading(new Pose2d(PARAMS.x1+PARAMS.lenBack*cos(toRadians(60)), PARAMS.y2-PARAMS.lenBack*sin(toRadians(60)), toRadians(PARAMS.angle)), toRadians(120))
            .strafeToLinearHeading(new Vector2d(PARAMS.x1, PARAMS.y2), toRadians(PARAMS.angle))
            .waitSeconds(1.5)
            .afterTime(0.3, bot.intake.setPower(0))
            .setTangent(toRadians(-90))
            .splineToSplineHeading(launchPose, toRadians(180))
            .afterTime(0, aimSequence())
            .waitSeconds(1.2)

            .afterTime(0, bot.intake.setPower(-1.0))
            .strafeToSplineHeading(new Vector2d(PARAMS.closeX, PARAMS.closeY1), toRadians(90))
            .strafeToConstantHeading(new Vector2d(PARAMS.closeX, PARAMS.closeY2))
            .afterTime(0.3, bot.intake.setPower(0))
            .strafeToSplineHeading(launchVec, toRadians(PARAMS.launchR))
            .afterTime(0, aimSequence())
            .waitSeconds(1.2)

            .build();

        waitForStart();

        //END OF INIT


        Actions.runBlocking(
            new SequentialAction(
                new EndAfterEitherParallel(
                    new ParallelAction(
                        RRPath,
                        new KeepRunning(bot.updatePoseAction()),
                        new KeepRunning(bot.canon.cloneMotorPower()),
                        new KeepRunning(bot.canon.setVelAction(launchSpeed))
                    ),
                    new Wait(29.6)
                ),
                new EndAfterFirstParallel(
                    new Wait(0.4),
                    new KeepRunning(bot.moveRelativeAction(1.0, 0.0, 0.0, 1.0))
                ),
                bot.stopAction(),
                bot.intake.setPower(0),
                bot.canon.setPowerAction(0)
            )
        );
    }

    public Action aimSequence() {
        return aimSequence(1.2, 0.1);
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
                    new SequentialAction(
                        new EndAfterFirstParallel(
                            new Wait(1.1),
                            new KeepRunning(bot.aimAtGoal())
                        ),
                        bot.stopAction()
                    ),
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


