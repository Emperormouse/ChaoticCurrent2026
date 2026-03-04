package org.firstinspires.ftc.teamcode.code.Auto;

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
public class AutoBLUE_GATE extends LinearOpMode {
    public static class PARAMS {
        public double x1 = 7.5;
        public double angle = -120;
        public double y1 = -50;
        public double y2 = -63.7;
        public double xMiddle = 10;
        public double yMiddle1 = -30;
        public double yMiddle2 = -60;
        public double closeX = -14;
        public double closeY1 = -33;
        public double closeY2 = -53.5;
        public double launchX = -15;
        public double launchY = -17;
        public double launchR = 38;
        public double gateX = 9;
        public double gateY = -58;
        public double gateR = 0;
    }
    public static PARAMS PARAMS = new PARAMS();

    MecanumDrive drive;
    Bot bot;
    Pose2d launchPose = new Pose2d(PARAMS.launchX, PARAMS.launchY, Math.toRadians(PARAMS.launchR));
    Vector2d launchVec = new Vector2d(launchPose.position.x, launchPose.position.y);
    double launchSpeed = 1440;

    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(-52.87, -36.2, Math.toRadians(-90)));
        bot = new Bot(hardwareMap, drive, Side.BLUE, telemetry);

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
            .strafeToLinearHeading(launchVec, toRadians(PARAMS.launchR))
            .afterTime(0, aimSequence(2.4, 1.1))
            .waitSeconds(2.4)

            .afterTime(0, bot.intake.setPower(-1.0))
            .setTangent(toRadians(0))
            .splineToSplineHeading(new Pose2d(PARAMS.xMiddle, PARAMS.yMiddle1, toRadians(-90)), toRadians(-90))
            .strafeToConstantHeading(new Vector2d(PARAMS.xMiddle, PARAMS.yMiddle2))
            .afterTime(0.3, bot.intake.setPower(0))
            .strafeToSplineHeading(new Vector2d(PARAMS.gateX, PARAMS.gateY), toRadians(PARAMS.gateR))
            .setTangent(toRadians(90))
            .splineToLinearHeading(launchPose, toRadians(180))
            .afterTime(0, aimSequence())
            .waitSeconds(1.3)

            .afterTime(0, bot.intake.setPower(-1.0))
            .setTangent(toRadians(0))
            .splineToSplineHeading(new Pose2d(PARAMS.x1, PARAMS.y1, toRadians(PARAMS.angle)), toRadians(-90))
            .strafeToLinearHeading(new Vector2d(PARAMS.x1, PARAMS.y2), toRadians(PARAMS.angle))
            .waitSeconds(1.5)
            .afterTime(0.3, bot.intake.setPower(0))
            .setTangent(toRadians(90))
            .splineToLinearHeading(launchPose, toRadians(180))
            .afterTime(0, aimSequence())
            .waitSeconds(1.3)

            .afterTime(0, bot.intake.setPower(-1.0))
            .setTangent(toRadians(0))
            .splineToSplineHeading(new Pose2d(PARAMS.x1, PARAMS.y1, toRadians(PARAMS.angle)), toRadians(-90))
            .strafeToLinearHeading(new Vector2d(PARAMS.x1, PARAMS.y2), toRadians(PARAMS.angle))
            .waitSeconds(1.5)
            .afterTime(0.3, bot.intake.setPower(0))
            .setTangent(toRadians(90))
            .splineToLinearHeading(launchPose, toRadians(180))
            .afterTime(0, aimSequence())
            .waitSeconds(1.3)

            .afterTime(0, bot.intake.setPower(-1.0))
            .strafeToSplineHeading(new Vector2d(PARAMS.closeX, PARAMS.closeY1), toRadians(-90))
            .lineToY(PARAMS.closeY2)
            .afterTime(0.3, bot.intake.setPower(0))
            .strafeToLinearHeading(launchVec, toRadians(PARAMS.launchR))
            .afterTime(0, aimSequence())
            .waitSeconds(1.3)

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
                    new KeepRunning(bot.moveRelativeAction(-1.0, 0.0, 0.0, 1.0))
                ),
                bot.stopAction(),
                bot.intake.setPower(0),
                bot.canon.setPowerAction(0)
            )
        );
    }

    public Action aimSequence() {
        return aimSequence(1.3, 0.0);
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


